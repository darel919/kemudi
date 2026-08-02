<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, shallowRef, watch } from 'vue'
import * as THREE from 'three'
import VehicleMesh from '~/components/VehicleMesh.vue'
import RemoteVehicleMesh from '~/components/RemoteVehicleMesh.vue'
import DrivingHud from '~/components/DrivingHud.vue'
import { useCameraSystem, type CameraMode } from '~/composables/useCameraSystem'
import type { SceneFrameCallback } from '~/composables/useThreeScene'
import { useInput } from '~/composables/useInput'
import { usePhysicsEngine } from '~/composables/usePhysicsEngine'
import { useVehicleLoader } from '~/composables/useVehicleLoader'
import { useTelemetryStore } from '~/stores/telemetry'
import type { InputState } from '~/composables/useInput'
import { VIRTUAL_OBD_PIDS } from '~/types/telemetry'
import type { BaseMapId } from '~/types/base-map'
import { getBaseMap } from '~/types/base-map'
import { useTerrain } from '~/composables/useTerrain'
import { useVehicleSessionStore } from '~/stores/vehicleSession'
import { useGraphicsSettings } from '~/composables/useGraphicsSettings'
import { TELEMETRY_LENGTH } from '~/types/physics'
import { useMultiplayer } from '~/composables/useMultiplayer'
import { useMultiplayerStore } from '~/stores/multiplayer'
import type { TransmissionMode } from '~/stores/vehicleSession'

const props = defineProps<{
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  vehiclePath: string
  vehicleName: string
  modeLabel: string
  transmission: 'manual' | 'automatic'
  mapId: BaseMapId
}>()

const emit = defineEmits<{
  menu: []
}>()

const loader = useVehicleLoader()
const physics = usePhysicsEngine()
const inputSystem = useInput()
const telemetry = useTelemetryStore()
const vehicleSession = useVehicleSessionStore()
const { effectivePreset } = useGraphicsSettings()
const multiplayer = useMultiplayer()
const multiplayerStore = useMultiplayerStore()
const cameraSystem = useCameraSystem(props.camera)
const baseMap = getBaseMap(props.mapId)
const terrain = useTerrain({
  width: baseMap.width,
  depth: baseMap.depth,
  segmentsW: effectivePreset.value === 'low' ? Math.min(baseMap.segments, 48) : effectivePreset.value === 'medium' ? Math.min(baseMap.segments, 72) : Math.min(baseMap.segments, 96),
  segmentsD: effectivePreset.value === 'low' ? Math.min(baseMap.segments, 48) : effectivePreset.value === 'medium' ? Math.min(baseMap.segments, 72) : Math.min(baseMap.segments, 96),
  heightScale: baseMap.heightScale,
  profile: baseMap.profile,
  color: baseMap.color,
  roughness: baseMap.roughness,
})

const restPositions = shallowRef(new Float64Array(0))
const bodyMaterial = ref('steel')
const bodyMeshPath = ref<string | undefined>(undefined)
const wheelRestLengths = ref<number[]>([])
const wheelRadii = ref<number[]>([])
const spawnLift = ref(0)
const physicsReady = ref(false)
const errorMessage = ref('')
const speed = ref(0)
const rpm = ref(800)
const gear = ref(1)
const engineTorqueCurve = shallowRef<readonly (readonly [number, number])[]>([])
const steeringAngle = ref(0)
const wheelSpeeds = [0, 0, 0, 0]
const cameraMode = ref<CameraMode>('exterior')
const transmissionMode = ref<TransmissionMode>(props.transmission)
const availableTransmissionModes = computed<TransmissionMode[]>(() =>
  vehicleSession.vehicle?.transmissionOptions?.length
    ? vehicleSession.vehicle.transmissionOptions
    : ['manual', 'automatic'],
)
const inputSnapshot = shallowRef<Readonly<InputState>>(inputSystem.state.value)
const EMPTY_POSITIONS = new Float64Array(0)
const renderPositions = computed(() =>
  (physics.positions.value ?? EMPTY_POSITIONS).length === restPositions.value.length && restPositions.value.length > 0
    ? (physics.positions.value ?? EMPTY_POSITIONS)
    : restPositions.value,
)
const obd = shallowRef({
  coolant: 25,
  oilPressure: 0,
  fuel: 0,
  grip: 0,
  damage: 0,
  abs: false,
  tc: false,
  fcw: false,
  aeb: false,
})
const remoteSnapshots = computed(() => Object.values(multiplayerStore.remoteSnapshots))
const vehicleCenter = new THREE.Vector3()
const telemetryUpdates: [string, number | string | boolean][] = []

let disposed = false
let gearUpLatch = false
let gearDownLatch = false
let ignitionLatch = false
let transmissionLatch = false
let baseGround: THREE.Object3D | null = null
let previousSpeedMps = 0
let lastRemotePruneAt = 0
let lastTelemetryPublishAt = 0
let lastTelemetryHistoryAt = 0
let lastInputPublishAt = 0
let registeredFrameCallback: { callbacks: Set<SceneFrameCallback>; callback: SceneFrameCallback } | null = null

const CAMERA_MODES: CameraMode[] = ['exterior', 'hood', 'grill', 'far-exterior', 'interior', 'cinematic', 'free']

function getCentroid(positions: Float64Array, center: THREE.Vector3): THREE.Vector3 {
  center.set(0, 0, 0)
  const count = positions.length / 3
  if (count === 0) return center
  for (let i = 0; i < positions.length; i += 3) {
    center.x += positions[i] ?? 0
    center.y += positions[i + 1] ?? 0
    center.z += positions[i + 2] ?? 0
  }
  return center.multiplyScalar(1 / count)
}

function getVehicleYaw(positions: Float64Array): number {
  if (positions.length < 12) return 0
  const frontX = ((positions[0] ?? 0) + (positions[3] ?? 0)) * 0.5
  const frontZ = ((positions[2] ?? 0) + (positions[5] ?? 0)) * 0.5
  const rearX = ((positions[6] ?? 0) + (positions[9] ?? 0)) * 0.5
  const rearZ = ((positions[8] ?? 0) + (positions[11] ?? 0)) * 0.5
  const dx = frontX - rearX
  const dz = frontZ - rearZ
  return Math.atan2(dx, -dz)
}

function getTerrainProfile(): 0 | 1 | 2 {
  if (baseMap.profile === 'bumpy') return 1
  if (baseMap.profile === 'offroad') return 2
  return 0
}

function updateTelemetry(snapshot: Float64Array, now: number) {
  const speedMps = snapshot[0] ?? 0
  speed.value = Math.max(0, snapshot[1] ?? 0)
  rpm.value = Math.max(0, snapshot[2] ?? 0)
  gear.value = Math.round(snapshot[3] ?? 0)
  obd.value = {
    coolant: snapshot[8] ?? 25,
    oilPressure: snapshot[10] ?? 0,
    fuel: snapshot[31] ?? 0,
    grip: snapshot[16] ?? 0,
    damage: snapshot[14] ?? 0,
    abs: (snapshot[22] ?? 0) > 0.5,
    tc: (snapshot[23] ?? 0) > 0.5,
    fcw: (snapshot[69] ?? 0) > 0.5,
    aeb: (snapshot[70] ?? 0) > 0.5,
  }
  const acceleration = Math.abs(speedMps - previousSpeedMps) / (1 / 60)
  previousSpeedMps = speedMps
  telemetryUpdates.length = 0
  telemetryUpdates.push(
    ['vehicle.speed', speed.value],
    ['vehicle.acceleration', acceleration],
    ['engine.rpm', rpm.value],
    ['drivetrain.gear', gear.value],
    ['drivetrain.clutch_engagement', (snapshot[7] ?? 0) * 100],
    ['drivetrain.torque_output', Math.abs(snapshot[19] ?? 0)],
    ['fuel.level', snapshot[31] ?? 0],
    ['engine.coolant_temp', snapshot[8] ?? 25],
    ['engine.oil_temp', snapshot[9] ?? 25],
    ['engine.oil_pressure', snapshot[10] ?? 0],
    ['drivetrain.transmission_temp', snapshot[11] ?? 25],
    ['drivetrain.tcm_fault_mask', snapshot[72] ?? 0],
    ['drivetrain.tcm_diagnostic_code', snapshot[73] ?? 0],
    ['drivetrain.tcm_input_speed', snapshot[74] ?? 0],
    ['drivetrain.tcm_output_speed', snapshot[75] ?? 0],
    ['drivetrain.tcm_shift_latency', snapshot[77] ?? 1],
    ['drivetrain.tcm_torque_reduction', (snapshot[78] ?? 0) * 100],
  )
  const wheelIds = ['fl', 'fr', 'rl', 'rr'] as const
  for (const [index, id] of wheelIds.entries()) {
    telemetryUpdates.push(
      [`vehicle.wheel_speed_${id}`, snapshot[44 + index] ?? 0],
      [`suspension.compression_${id}`, snapshot[48 + index] ?? 0],
      [`suspension.load_${id}`, snapshot[52 + index] ?? 0],
      [`tire.temp_${id}`, snapshot[56 + index] ?? 20],
      [`tire.wear_${id}`, snapshot[60 + index] ?? 0],
    )
  }
  telemetry.updateSnapshot(telemetryUpdates, Date.now(), now - lastTelemetryHistoryAt >= 500)
  if (now - lastTelemetryHistoryAt >= 500) lastTelemetryHistoryAt = now
}

function getAdasTarget(center: THREE.Vector3, vehicleYaw: number): { distance: number; relativeSpeed: number } {
  let closest = Number.POSITIVE_INFINITY
  let relativeSpeed = 0
  const forwardX = Math.sin(vehicleYaw)
  const forwardZ = -Math.cos(vehicleYaw)
  const rightX = Math.cos(vehicleYaw)
  const rightZ = Math.sin(vehicleYaw)
  for (const snapshot of remoteSnapshots.value) {
    if (snapshot.positions.length < 3) continue
    let x = 0; let y = 0; let z = 0; let count = 0
    for (let i = 0; i + 2 < snapshot.positions.length; i += 3) {
      x += snapshot.positions[i] ?? 0
      y += snapshot.positions[i + 1] ?? 0
      z += snapshot.positions[i + 2] ?? 0
      count++
    }
    if (!count) continue
    const dx = x / count - center.x
    const dz = z / count - center.z
    const forwardDistance = dx * forwardX + dz * forwardZ
    const lateralDistance = dx * rightX + dz * rightZ
    if (forwardDistance <= 0 || Math.abs(lateralDistance) > 2.5) continue
    const distance = Math.sqrt(dx * dx + dz * dz)
    if (distance < closest) {
      closest = distance
      relativeSpeed = speed.value / 3.6 - (snapshot.telemetry[0] ?? 0)
    }
  }
  return Number.isFinite(closest) ? { distance: closest, relativeSpeed } : { distance: 0, relativeSpeed: 0 }
}

function tick(now: number, dtMs: number) {
  if (disposed) return
  const dt = Math.min(0.05, Math.max(1 / 240, dtMs / 1000))
  if (now - lastRemotePruneAt >= 1000) {
    multiplayerStore.pruneStaleSnapshots(Date.now())
    lastRemotePruneAt = now
  }

  inputSystem.pollGamepad()
  const input = inputSystem.state.value
  if (now - lastInputPublishAt >= 100) {
    inputSnapshot.value = { ...input }
    lastInputPublishAt = now
  }
  if (input.ignitionToggle && !ignitionLatch) vehicleSession.advanceIgnition()
  ignitionLatch = input.ignitionToggle
  if (input.transmissionToggle && !transmissionLatch) {
    const nextMode: TransmissionMode = transmissionMode.value === 'manual' ? 'automatic' : 'manual'
    if (availableTransmissionModes.value.includes(nextMode)) {
      transmissionMode.value = nextMode
      vehicleSession.setTransmissionMode(nextMode)
      physics.setTransmissionMode(nextMode)
    }
  }
  transmissionLatch = input.transmissionToggle
  const engineRunning = vehicleSession.ignition === 'running'
  getCentroid(renderPositions.value, vehicleCenter)
  const vehicleYaw = getVehicleYaw(renderPositions.value)
  const adasTarget = getAdasTarget(vehicleCenter, vehicleYaw)
  if (physicsReady.value) {
    physics.step(Math.min(0.05, Math.max(1 / 240, dt)), {
      steering: input.steering,
      throttle: input.throttle,
      brake: input.brake,
      clutch: input.clutch,
      handbrake: input.handbrake,
      gearUp: input.gearUp && !gearUpLatch,
      gearDown: input.gearDown && !gearDownLatch,
      engineOn: engineRunning,
      adasTargetDistance: adasTarget.distance,
      adasTargetRelativeSpeed: adasTarget.relativeSpeed,
    })
    const physicsTelemetry = physics.telemetry.value ?? new Float64Array(TELEMETRY_LENGTH)
    steeringAngle.value = physicsTelemetry[6] ?? 0
    for (let index = 0; index < wheelSpeeds.length; index++) {
      wheelSpeeds[index] = (physicsTelemetry[44 + index] ?? 0) / 3.6
    }
    gearUpLatch = input.gearUp
    gearDownLatch = input.gearDown
    if (now - lastTelemetryPublishAt >= 100) {
      updateTelemetry(physics.telemetry.value ?? new Float64Array(TELEMETRY_LENGTH), now)
      lastTelemetryPublishAt = now
    }
    multiplayer.sendSnapshot(
      props.vehicleName,
      physics.positions.value ?? EMPTY_POSITIONS,
      physics.telemetry.value ?? new Float64Array(TELEMETRY_LENGTH),
    )
  }

  const center = vehicleCenter
  cameraSystem.update(center, vehicleYaw, dt * 1000)
}

function cycleCamera() {
  const current = CAMERA_MODES.indexOf(cameraMode.value)
  const next = CAMERA_MODES[(current + 1) % CAMERA_MODES.length] ?? 'exterior'
  cameraMode.value = next
  cameraSystem.setMode(next)
}

async function loadVehicle() {
  try {
    const vehicle = await loader.loadFromUrl(props.vehiclePath)
    bodyMaterial.value = vehicle.body?.material ?? 'steel'
    bodyMeshPath.value = vehicle.body?.bodyMesh
    // Vehicle files describe the wheel mounts at local y=0. Lift the whole
    // assembly to the suspension-mount height. The first four physics nodes
    // are wheel mounts; the worker's raycast subtracts rest length and tire
    // radius from them to find terrain contact.
    const definition = loader.toPhysicsDefinition(vehicle)
    engineTorqueCurve.value = definition.engine?.torqueCurve ?? []
    const wheelConfigs = definition.suspension?.wheels ?? []
    const vehicleLift = Math.max(
      0.34,
      ...wheelConfigs.slice(0, 4).map((wheel, index) => {
        const node = definition.nodes[index]
        const terrainY = node ? terrain.getHeightAt(node.x, node.z) : 0
        return terrainY + wheel.restLength + wheel.tireRadius - (node?.y ?? 0)
      }),
    )
    spawnLift.value = vehicleLift
    wheelRestLengths.value = wheelConfigs.slice(0, 4).map(wheel => wheel.restLength)
    wheelRadii.value = wheelConfigs.slice(0, 4).map(wheel => wheel.tireRadius)
    for (const node of definition.nodes) node.y += vehicleLift
    const rest = new Float64Array(definition.nodes.length * 3)
    for (const [index, node] of definition.nodes.entries()) {
      rest[index * 3] = node.x
      rest[index * 3 + 1] = node.y
      rest[index * 3 + 2] = node.z
    }
    restPositions.value = rest
    telemetry.registerAllSignals(VIRTUAL_OBD_PIDS)
    await physics.loadVehicle(definition, getTerrainProfile())
    physics.setTransmissionMode(transmissionMode.value)
    physicsReady.value = true
  } catch (error) {
    errorMessage.value = error instanceof Error ? error.message : 'Unable to initialize vehicle physics'
  }
}

onMounted(async () => {
  baseGround = props.scene.children.find(child => child.userData.kemudiBaseGround === true) ?? null
  if (baseGround) baseGround.visible = false
  props.scene.add(terrain.mesh)
  props.scene.add(terrain.decorations)
  inputSystem.init()
  cameraSystem.setMode('exterior')
  await loadVehicle()
  if (multiplayerStore.multiplayerEnabled && multiplayerStore.configuredServerUrl) {
    multiplayer.connect(
      multiplayerStore.configuredServerUrl,
      multiplayerStore.configuredRoomId,
      props.vehicleName.replace(/[^A-Za-z0-9_-]/g, '-').slice(0, 64) || 'vehicle',
    )
  }
  if (!disposed) {
    const callbacks = props.scene.userData.kemudiFrameCallbacks as Set<SceneFrameCallback> | undefined
    if (callbacks) {
      const callback: SceneFrameCallback = tick
      callbacks.add(callback)
      registeredFrameCallback = { callbacks, callback }
    }
  }
})

onBeforeUnmount(() => {
  disposed = true
  if (registeredFrameCallback) {
    registeredFrameCallback.callbacks.delete(registeredFrameCallback.callback)
    registeredFrameCallback = null
  }
  inputSystem.dispose()
  physics.dispose()
  multiplayer.disconnect()
  cameraSystem.dispose()
  props.scene.remove(terrain.mesh)
  props.scene.remove(terrain.decorations)
  terrain.dispose()
  if (baseGround) baseGround.visible = true
})

watch(() => props.transmission, (mode) => {
  transmissionMode.value = mode
  physics.setTransmissionMode(mode)
})
</script>

<template>
  <VehicleMesh
    v-if="restPositions.length > 0"
    :scene="scene"
    :positions="renderPositions"
    :rest-positions="restPositions"
    :quality="effectivePreset"
    :body-material="bodyMaterial"
    :body-mesh-path="bodyMeshPath"
    :wheel-rest-lengths="wheelRestLengths"
    :wheel-radii="wheelRadii"
    :steering-angle="steeringAngle"
    :wheel-speeds="wheelSpeeds"
    :spawn-lift="spawnLift"
    :terrain-height-at="terrain.getHeightAt"
  />
  <RemoteVehicleMesh
    v-for="snapshot in remoteSnapshots"
    :key="snapshot.id"
    :scene="scene"
    :positions="snapshot.positions"
  />
  <DrivingHud
    :vehicle-name="vehicleName"
    :mode-label="modeLabel"
    :map-label="baseMap.label"
    :speed="speed"
    :rpm="rpm"
    :gear="gear"
    :transmission-mode="transmissionMode"
    :transmission-toggle-available="availableTransmissionModes.length > 1"
    :camera-mode="cameraMode"
    :ignition-state="vehicleSession.ignition"
    :input="inputSnapshot"
    :obd="obd"
    :engine-torque-curve="engineTorqueCurve"
    @camera="cycleCamera"
    @menu="emit('menu')"
  />
  <div v-if="errorMessage" class="vehicle-error" role="alert">{{ errorMessage }}</div>
</template>

<style scoped>
.vehicle-error { position: absolute; z-index: 8; top: 82px; left: 20px; max-width: min(420px, calc(100vw - 40px)); padding: 10px 13px; color: #ffd6d6; background: rgba(101, 20, 32, .86); border: 1px solid #c85d6b; border-radius: 7px; font: 12px/1.4 ui-sans-serif, system-ui, sans-serif; }
</style>
