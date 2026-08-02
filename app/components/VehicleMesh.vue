<script setup lang="ts">
import { onBeforeUnmount, onMounted, shallowRef, watch } from 'vue'
import * as THREE from 'three'
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js'
import {
  alignGeometryToChassisFootprint,
  CHASSIS_FRAME_SIZE,
  computeChassisFrame,
  useVehicleSkinning,
} from '~/composables/useVehicleSkinning'
import { logDebug } from '~/utils/debug'
import { applyWheelOrientation, computeVisualAckermannAngles, computeVisualWheelY } from '~/utils/vehicleWheelTransforms'

const props = defineProps<{
  positions: Float64Array
  restPositions: Float64Array
  scene: THREE.Scene
  color?: number
  quality?: 'low' | 'medium' | 'high'
  bodyMaterial?: string
  bodyMeshPath?: string
  wheelRestLengths?: number[]
  wheelTravels?: number[]
  wheelRadii?: number[]
  /** Authoritative front-wheel steering angle in radians. */
  steeringAngle?: number
  /** Per-wheel linear rolling speed in m/s, used only for visual spin. */
  wheelSpeeds?: number[]
  /** Authoritative per-wheel suspension compression in [0, 1]. */
  wheelCompressions?: number[]
  /** World-space lift applied to the vehicle definition at spawn. */
  spawnLift?: number
  terrainHeightAt?: (x: number, z: number) => number
}>()

const meshRef = shallowRef<THREE.Mesh | null>(null)
let disposed = false
let bodyGeometry: THREE.BufferGeometry | null = null
let bodyMaterial: THREE.MeshStandardMaterial | null = null
let bodyMesh: THREE.Mesh | null = null
let lowBodyMesh: THREE.Mesh | null = null
let bodyLod: THREE.LOD | null = null
let usesAuthoredBody = false
let assembly: THREE.Group | null = null
let wheelGeometry: THREE.CylinderGeometry | null = null
let wheelMaterial: THREE.MeshStandardMaterial | null = null
let wheels: THREE.Mesh[] = []
let wheelPivots: THREE.Group[] = []
let wheelSpinPivots: THREE.Group[] = []
let skinning: ReturnType<typeof useVehicleSkinning> | null = null
let visualUpdateCount = 0
let lastVisualUpdateAt = 0
const wheelSpinAngles = [0, 0, 0, 0]
const boundsMin = new THREE.Vector3()
const boundsMax = new THREE.Vector3()
const boundsPoint = new THREE.Vector3()
const chassisFrame = new Float64Array(CHASSIS_FRAME_SIZE)
const chassisRight = new THREE.Vector3()
const chassisUp = new THREE.Vector3()
const chassisBackward = new THREE.Vector3()
const chassisRotationMatrix = new THREE.Matrix4()
const chassisOrientation = new THREE.Quaternion()
const frontSteeringAngles = new Float64Array(2)
let visualWheelbase = 2.5
let visualTrackWidth = 1.5

function configureVisualSteeringGeometry() {
  if (props.restPositions.length < 12) return
  const frontX = ((props.restPositions[0] ?? 0) + (props.restPositions[3] ?? 0)) * 0.5
  const frontZ = ((props.restPositions[2] ?? 0) + (props.restPositions[5] ?? 0)) * 0.5
  const rearX = ((props.restPositions[6] ?? 0) + (props.restPositions[9] ?? 0)) * 0.5
  const rearZ = ((props.restPositions[8] ?? 0) + (props.restPositions[11] ?? 0)) * 0.5
  const frontTrack = Math.hypot(
    (props.restPositions[3] ?? 0) - (props.restPositions[0] ?? 0),
    (props.restPositions[5] ?? 0) - (props.restPositions[2] ?? 0),
  )
  const rearTrack = Math.hypot(
    (props.restPositions[9] ?? 0) - (props.restPositions[6] ?? 0),
    (props.restPositions[11] ?? 0) - (props.restPositions[8] ?? 0),
  )
  visualWheelbase = Math.max(0.1, Math.hypot(frontX - rearX, frontZ - rearZ))
  visualTrackWidth = Math.max(0.1, (frontTrack + rearTrack) * 0.5)
}

function updateChassisOrientation(positions: Float64Array) {
  if (!computeChassisFrame(chassisFrame, positions)) {
    chassisOrientation.identity()
    return
  }
  chassisRight.set(chassisFrame[3] ?? 1, chassisFrame[4] ?? 0, chassisFrame[5] ?? 0)
  chassisUp.set(chassisFrame[6] ?? 0, chassisFrame[7] ?? 1, chassisFrame[8] ?? 0)
  // The chassis frame stores vehicle-forward, which is authored local -Z.
  chassisBackward.set(-(chassisFrame[9] ?? 0), -(chassisFrame[10] ?? 0), -(chassisFrame[11] ?? -1))
  chassisRotationMatrix.makeBasis(chassisRight, chassisUp, chassisBackward)
  chassisOrientation.setFromRotationMatrix(chassisRotationMatrix).normalize()
}

function getBounds(positions: Float64Array) {
  boundsMin.set(Infinity, Infinity, Infinity)
  boundsMax.set(-Infinity, -Infinity, -Infinity)
  for (let i = 0; i < positions.length; i += 3) {
    boundsPoint.set(positions[i] ?? 0, positions[i + 1] ?? 0, positions[i + 2] ?? 0)
    boundsMin.min(boundsPoint)
    boundsMax.max(boundsPoint)
  }
  return { min: boundsMin, max: boundsMax }
}

function createBoxBody() {
  const { min, max } = getBounds(props.restPositions)
  const size = new THREE.Vector3(
    Math.max(0.5, max.x - min.x),
    Math.max(0.35, max.y - min.y),
    Math.max(0.8, max.z - min.z),
  )
  const center = new THREE.Vector3().addVectors(min, max).multiplyScalar(0.5)

  const materialColors: Record<string, number> = {
    steel: 0x318fc7,
    aluminum: 0x92a8b8,
    fiberglass: 0xd9783f,
    carbon_fiber: 0x28313b,
    composite: 0x5a9d78,
  }
  const bodyColor = props.color ?? materialColors[props.bodyMaterial ?? ''] ?? 0x318fc7

  bodyGeometry = new THREE.BoxGeometry(size.x, size.y, size.z, 2, 1, 2)
  bodyGeometry.translate(center.x, center.y, center.z)
  bodyGeometry.computeVertexNormals()
  bodyMaterial = new THREE.MeshStandardMaterial({
    color: bodyColor,
    roughness: 0.38,
    metalness: 0.55,
    side: THREE.DoubleSide,
  })
  bodyMesh = new THREE.Mesh(bodyGeometry, bodyMaterial)
  bodyMesh.frustumCulled = false
  bodyMesh.castShadow = props.quality !== 'low'
  bodyMesh.receiveShadow = props.quality !== 'low'
  // This fallback box is already tiny. Reuse its skinned geometry at every
  // quality level so low/distant rendering cannot drop chassis rotation by
  // switching to a centroid-only proxy.
  lowBodyMesh = null
}

function createGltfBody(geometry: THREE.BufferGeometry) {
  const materialColors: Record<string, number> = {
    steel: 0x318fc7,
    aluminum: 0x92a8b8,
    fiberglass: 0xd9783f,
    carbon_fiber: 0x28313b,
    composite: 0x5a9d78,
  }
  const bodyColor = props.color ?? materialColors[props.bodyMaterial ?? ''] ?? 0x318fc7

  bodyGeometry = geometry
  bodyMaterial = new THREE.MeshStandardMaterial({
    color: bodyColor,
    roughness: 0.38,
    metalness: 0.55,
  })
  bodyMesh = new THREE.Mesh(bodyGeometry, bodyMaterial)
  usesAuthoredBody = true
  bodyMesh.castShadow = props.quality !== 'low'
  bodyMesh.receiveShadow = props.quality !== 'low'
}

async function loadGltfBodyMesh(url: string): Promise<THREE.BufferGeometry | null> {
  const loader = new GLTFLoader()
  try {
    const gltf = await loader.loadAsync(url)
    gltf.scene.updateMatrixWorld(true)
    // Find the first mesh with a BufferGeometry
    let result: THREE.BufferGeometry | null = null
    gltf.scene.traverse((child) => {
      if (result) return
      if (child instanceof THREE.Mesh && child.geometry) {
        const geometry = child.geometry.clone()
        // GLB meshes may carry their authored placement on the node rather
        // than in vertex data. Bake that transform before comparing the mesh
        // to world-space physics nodes; otherwise the shell can spawn at the
        // origin while the wheels are correctly lifted onto the terrain.
        geometry.applyMatrix4(child.matrixWorld)
        result = geometry
      }
    })
    if (!result) {
      logDebug('vehicle-mesh:gltf-no-mesh', { url })
      return null
    }
    const vertexCount = (result as THREE.BufferGeometry).attributes.position?.count ?? 0
    logDebug('vehicle-mesh:gltf-loaded', { url, vertexCount })
    return result
  } catch (err) {
    logDebug('vehicle-mesh:gltf-error', { url, error: String(err) })
    return null
  }
}

function assembleBody() {
  assembly = new THREE.Group()
  assembly.name = 'kemudi-vehicle-assembly'
  bodyLod = new THREE.LOD()
  if (usesAuthoredBody && bodyMesh) {
    // Authored GLB geometry remains active on every graphics preset. The
    // generated box is only a failure fallback; using it as a distance LOD
    // makes valid vehicle assets look like boxes on low/auto-quality devices.
    bodyLod.addLevel(bodyMesh, 0)
  } else if (props.quality === 'low') {
    bodyLod.addLevel(lowBodyMesh ?? bodyMesh!, 0)
  } else {
    bodyLod.addLevel(bodyMesh!, 0)
    if (lowBodyMesh) bodyLod.addLevel(lowBodyMesh, props.quality === 'medium' ? 18 : 32)
  }
  const spawnLift = usesAuthoredBody
    ? props.spawnLift ?? Math.max(0.34, ...(props.wheelRestLengths ?? []).map((length, index) => length + (props.wheelRadii?.[index] ?? 0)))
    : 0
  // Physics nodes are lifted from the vehicle file's local wheel-mount plane
  // to the terrain at spawn. Move authored geometry into that same world
  // space before computing skin weights; an offset on the LOD alone makes
  // weights compare local GLB vertices to elevated physics nodes and causes
  // the body shell to drift away from the wheels under load.
  if (usesAuthoredBody && bodyGeometry) {
    alignGeometryToChassisFootprint(bodyGeometry, props.restPositions)
    if (spawnLift !== 0) bodyGeometry.translate(0, spawnLift, 0)
  }
  bodyGeometry?.computeVertexNormals()
  bodyLod.position.set(0, 0, 0)
  meshRef.value = bodyMesh
  skinning = useVehicleSkinning(bodyGeometry!, props.restPositions)
  assembly.add(bodyLod)
  configureVisualSteeringGeometry()

  const wheelSegments = props.quality === 'low' ? 12 : props.quality === 'medium' ? 16 : 20
  const wheelRadius = props.wheelRadii?.[0] ?? 0.18
  wheelGeometry = new THREE.CylinderGeometry(
    wheelRadius,
    wheelRadius,
    Math.max(0.12, wheelRadius * 0.42),
    wheelSegments,
  )
  wheelMaterial = new THREE.MeshStandardMaterial({ color: 0x101722, roughness: 0.82, metalness: 0.12 })
  const wheelCount = Math.min(4, props.restPositions.length / 3)
  for (let i = 0; i < wheelCount; i++) {
    const wheelPivot = new THREE.Group()
    wheelPivot.name = `wheel-${i}-steering-pivot`
    const wheelSpinPivot = new THREE.Group()
    wheelSpinPivot.name = `wheel-${i}-spin-pivot`
    const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial)
    wheel.rotation.z = Math.PI / 2
    wheel.castShadow = props.quality !== 'low'
    wheel.receiveShadow = false
    wheelSpinPivot.add(wheel)
    wheelPivot.add(wheelSpinPivot)
    assembly.add(wheelPivot)
    wheels.push(wheel)
    wheelPivots.push(wheelPivot)
    wheelSpinPivots.push(wheelSpinPivot)
  }

  props.scene.add(assembly)

  logDebug('vehicle-mesh:mounted', {
    nodeCount: props.restPositions.length / 3,
    wheelCount,
    bodySource: usesAuthoredBody ? 'gltf' : 'box',
  })
  updateVisuals(props.positions)
}

async function createBody() {
  if (props.bodyMeshPath) {
    const geometry = await loadGltfBodyMesh(props.bodyMeshPath)
    if (disposed) return
    if (geometry) {
      createGltfBody(geometry)
      assembleBody()
      return
    }
    // Fall through to box on GLTF failure
    logDebug('vehicle-mesh:falling-back-to-box', { path: props.bodyMeshPath })
  }
  createBoxBody()
  assembleBody()
}

function updateVisuals(positions: Float64Array) {
  if (disposed || positions.length !== props.restPositions.length) return
  const now = globalThis.performance?.now() ?? Date.now()
  const visualDt = lastVisualUpdateAt > 0
    ? Math.min(0.1, Math.max(0, (now - lastVisualUpdateAt) / 1000))
    : 0
  lastVisualUpdateAt = now
  skinning?.update(positions)
  visualUpdateCount += 1
  // Keep deformation responsive without rebuilding dynamic normals and bounds
  // for every worker message on the main thread.
  if (visualUpdateCount % 2 === 0) {
    bodyGeometry?.computeVertexNormals()
    bodyGeometry?.computeBoundingSphere()
  }
  if (lowBodyMesh) {
    let x = 0; let y = 0; let z = 0
    for (let i = 0; i < positions.length; i += 3) {
      x += positions[i] ?? 0; y += positions[i + 1] ?? 0; z += positions[i + 2] ?? 0
    }
    const count = Math.max(1, positions.length / 3)
    lowBodyMesh.position.set(x / count, y / count, z / count)
  }

  updateChassisOrientation(positions)
  const steering = Number.isFinite(props.steeringAngle) ? props.steeringAngle! : 0
  computeVisualAckermannAngles(
    frontSteeringAngles,
    steering,
    visualWheelbase,
    visualTrackWidth,
  )

  for (let i = 0; i < wheels.length; i++) {
    const offset = i * 3
    const x = positions[offset] ?? 0
    const mountY = positions[offset + 1] ?? 0
    const z = positions[offset + 2] ?? 0
    const radius = props.wheelRadii?.[i] ?? props.wheelRadii?.[0] ?? 0.18
    const restLength = props.wheelRestLengths?.[i] ?? props.wheelRestLengths?.[0] ?? 0
    const travel = props.wheelTravels?.[i] ?? props.wheelTravels?.[0] ?? 0
    const compression = props.wheelCompressions?.[i]
    const terrainY = props.terrainHeightAt?.(x, z)
    // Suspension compression is calculated from the same terrain sample and
    // mount node in WASM. Use it as the visual authority so dynamic ruts and
    // any renderer/physics terrain sampling differences cannot separate the
    // tire from the chassis. Keep the terrain sample for the pre-telemetry
    // first frame and older callers that do not provide compression.
    const authoritativeWheelY = computeVisualWheelY(mountY, restLength, travel, compression)
    const wheelY = authoritativeWheelY ?? (() => {
      const canReachTerrain = terrainY !== undefined
        && mountY - terrainY - radius <= restLength + 0.02
      return canReachTerrain ? terrainY! + radius : mountY - restLength
    })()
    const wheelPivot = wheelPivots[i]
    const wheelSpeed = props.wheelSpeeds?.[i] ?? 0
    const safeWheelSpeed = Number.isFinite(wheelSpeed) ? wheelSpeed : 0
    const safeRadius = Math.max(0.05, radius)
    wheelSpinAngles[i] = (wheelSpinAngles[i] ?? 0) - safeWheelSpeed / safeRadius * visualDt
    if (wheelPivot) {
      applyWheelOrientation(wheelPivot, chassisOrientation, i, frontSteeringAngles)
      const wheelSpinPivot = wheelSpinPivots[i]
      if (wheelSpinPivot) wheelSpinPivot.rotation.x = wheelSpinAngles[i] ?? 0
    }
    wheelPivot?.position.set(
      x,
      wheelY,
      z,
    )
  }
}

watch(
  () => [props.positions, props.steeringAngle, props.wheelSpeeds, props.wheelCompressions],
  () => updateVisuals(props.positions),
  { immediate: true },
)

onMounted(createBody)

onBeforeUnmount(() => {
  if (disposed) return
  disposed = true
  if (assembly) props.scene.remove(assembly)
  skinning?.dispose()
  bodyGeometry?.dispose()
  bodyMaterial?.dispose()
  lowBodyMesh?.geometry.dispose()
  if (lowBodyMesh?.material instanceof THREE.Material) lowBodyMesh.material.dispose()
  wheelGeometry?.dispose()
  wheelMaterial?.dispose()
  wheels = []
  wheelPivots = []
  wheelSpinPivots = []
  assembly = null
  lastVisualUpdateAt = 0
  meshRef.value = null
  bodyLod = null
  lowBodyMesh = null
  usesAuthoredBody = false
  logDebug('vehicle-mesh:unmounted', {})
})

defineExpose({ mesh: meshRef })
</script>

<template />
