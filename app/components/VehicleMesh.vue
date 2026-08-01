<script setup lang="ts">
import { onBeforeUnmount, onMounted, shallowRef, watch } from 'vue'
import * as THREE from 'three'
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js'
import { useVehicleSkinning } from '~/composables/useVehicleSkinning'
import { logDebug } from '~/utils/debug'

const props = defineProps<{
  positions: Float64Array
  restPositions: Float64Array
  scene: THREE.Scene
  color?: number
  quality?: 'low' | 'medium' | 'high'
  bodyMaterial?: string
  bodyMeshPath?: string
  wheelRestLengths?: number[]
  wheelRadii?: number[]
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
let wheelGeometry: THREE.CylinderGeometry | null = null
let wheelMaterial: THREE.MeshStandardMaterial | null = null
let wheels: THREE.Mesh[] = []
let skinning: ReturnType<typeof useVehicleSkinning> | null = null
let visualUpdateCount = 0
const boundsMin = new THREE.Vector3()
const boundsMax = new THREE.Vector3()
const boundsPoint = new THREE.Vector3()

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
  })
  bodyMesh = new THREE.Mesh(bodyGeometry, bodyMaterial)
  bodyMesh.castShadow = props.quality !== 'low'
  bodyMesh.receiveShadow = props.quality !== 'low'
  lowBodyMesh = new THREE.Mesh(
    new THREE.BoxGeometry(Math.max(0.5, size.x), Math.max(0.35, size.y), Math.max(0.8, size.z), 1, 1, 1),
    new THREE.MeshStandardMaterial({ color: bodyColor, roughness: 0.65, metalness: 0.25 }),
  )
  lowBodyMesh.position.copy(center)
  lowBodyMesh.castShadow = false
  lowBodyMesh.receiveShadow = true
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
    // Find the first mesh with a BufferGeometry
    let result: THREE.BufferGeometry | null = null
    gltf.scene.traverse((child) => {
      if (result) return
      if (child instanceof THREE.Mesh && child.geometry) {
        result = child.geometry.clone()
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
  if (usesAuthoredBody && bodyGeometry && spawnLift !== 0) {
    bodyGeometry.translate(0, spawnLift, 0)
  }
  bodyGeometry?.computeVertexNormals()
  bodyLod.position.set(0, 0, 0)
  meshRef.value = bodyMesh
  skinning = useVehicleSkinning(bodyGeometry!, props.restPositions)
  props.scene.add(bodyLod)

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
    const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial)
    wheel.rotation.z = Math.PI / 2
    wheel.castShadow = props.quality !== 'low'
    wheel.receiveShadow = false
    wheels.push(wheel)
    props.scene.add(wheel)
  }

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
  skinning?.update(positions)
  visualUpdateCount += 1
  // Keep deformation responsive without rebuilding dynamic normals and bounds
  // for every worker message on the main thread.
  if (visualUpdateCount % 2 === 0) bodyGeometry?.computeVertexNormals()
  if (lowBodyMesh) {
    let x = 0; let y = 0; let z = 0
    for (let i = 0; i < positions.length; i += 3) {
      x += positions[i] ?? 0; y += positions[i + 1] ?? 0; z += positions[i + 2] ?? 0
    }
    const count = Math.max(1, positions.length / 3)
    lowBodyMesh.position.set(x / count, y / count, z / count)
  }

  for (let i = 0; i < wheels.length; i++) {
    const offset = i * 3
    const x = positions[offset] ?? 0
    const mountY = positions[offset + 1] ?? 0
    const z = positions[offset + 2] ?? 0
    const radius = props.wheelRadii?.[i] ?? props.wheelRadii?.[0] ?? 0.18
    const restLength = props.wheelRestLengths?.[i] ?? props.wheelRestLengths?.[0] ?? 0
    const terrainY = props.terrainHeightAt?.(x, z)
    const canReachTerrain = terrainY !== undefined
      && mountY - terrainY - radius <= restLength + 0.02
    const wheelY = canReachTerrain
      ? terrainY! + radius
      : mountY - restLength
    wheels[i]?.position.set(
      x,
      wheelY,
      z,
    )
  }
}

watch(() => props.positions, updateVisuals, { immediate: true })

onMounted(createBody)

onBeforeUnmount(() => {
  if (disposed) return
  disposed = true
  if (bodyLod) props.scene.remove(bodyLod)
  for (const wheel of wheels) props.scene.remove(wheel)
  skinning?.dispose()
  bodyGeometry?.dispose()
  bodyMaterial?.dispose()
  lowBodyMesh?.geometry.dispose()
  if (lowBodyMesh?.material instanceof THREE.Material) lowBodyMesh.material.dispose()
  wheelGeometry?.dispose()
  wheelMaterial?.dispose()
  wheels = []
  meshRef.value = null
  bodyLod = null
  lowBodyMesh = null
  usesAuthoredBody = false
  logDebug('vehicle-mesh:unmounted', {})
})

defineExpose({ mesh: meshRef })
</script>

<template />
