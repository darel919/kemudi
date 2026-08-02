import { ref, shallowRef, readonly } from 'vue'
import type {
  PhysicsInMessage,
  PhysicsOutMessage,
  PhysicsControls,
  TerrainProfileId,
  VehicleDefinition,
} from '~/types/physics'
import { TELEMETRY_LENGTH } from '~/types/physics'
import { logDebug } from '~/utils/debug'

export interface PhysicsEngineHandle {
  /** Whether the WASM worker is ready to accept steps */
  ready: ReturnType<typeof readonly>
  /** Current node positions [x0,y0,z0, x1,y1,z1, ...] */
  positions: ReturnType<typeof shallowRef<Float64Array>>
  /** Current node velocities [vx0,vy0,vz0, ...] */
  velocities: ReturnType<typeof shallowRef<Float64Array>>
  /** Fixed-width authoritative telemetry snapshot from WASM */
  telemetry: ReturnType<typeof shallowRef<Float64Array>>
  /** Current simulation time in seconds */
  time: ReturnType<typeof readonly>
  /** Initialize the WASM physics engine */
  init(): Promise<void>
  /** Load a vehicle definition into the engine */
  loadVehicle(vehicle: VehicleDefinition, terrainProfile?: TerrainProfileId, opts?: { groundFriction?: number; surfaceRoughness?: number; surfaceMoisture?: number; surfaceCompactness?: number; surfacePresetIndex?: number }): Promise<void>
  /** Switch the driver's manual/automatic gearbox mode without reloading. */
  setTransmissionMode(mode: 'manual' | 'automatic'): void
  /** Inject a deterministic TCM fault for diagnostics/scenario testing. */
  setTcmFault(faultId: number, active: boolean, intermittent?: boolean, seed?: number): void
  /** Clear TCM fault state and diagnostic codes without repairing wear. */
  clearTcmFaults(): void
  /** Restart a failed worker and restore the last loaded vehicle. */
  restart(): Promise<void>
  /** Step the simulation forward by dt seconds */
  step(dt: number, controls?: PhysicsControls): void
  /** Apply a force to a specific node */
  applyForce(nodeId: number, fx: number, fy: number, fz: number): void
  /** Reset the physics engine */
  reset(): void
  /** Dispose worker and free resources */
  dispose(): void
}

export function usePhysicsEngine(): PhysicsEngineHandle {
  const ready = ref(false)
  const time = ref(0)
  const positions = shallowRef<Float64Array>(new Float64Array(0))
  const velocities = shallowRef<Float64Array>(new Float64Array(0))
  const telemetry = shallowRef<Float64Array>(new Float64Array(TELEMETRY_LENGTH))

  let worker: Worker | null = null
  const pendingResolves: Array<() => void> = []
  const pendingRejects: Array<(err: Error) => void> = []
  let stepInFlight = false
  let queuedStepDt = 0
  let queuedControls: PhysicsControls | null = null
  let lastVehicle: VehicleDefinition | null = null
  let lastTerrainProfile: TerrainProfileId = 0
  let disposed = false

  function settleReady(error?: Error) {
    if (error) {
      while (pendingRejects.length) pendingRejects.shift()!(error)
      pendingResolves.length = 0
      return
    }
    while (pendingResolves.length) pendingResolves.shift()!()
    pendingRejects.length = 0
  }

  function dispatchStep(dt: number, controls: PhysicsControls) {
    if (!worker || disposed || !ready.value || stepInFlight) return
    stepInFlight = true
    sendMessage({ type: 'step', dt, controls })
  }

  function handleMessage(e: MessageEvent<PhysicsOutMessage>) {
    const msg = e.data
    switch (msg.type) {
      case 'ready':
        ready.value = true
        logDebug('physics-worker:initialized', { nodeCount: 0 })
        settleReady()
        break

      case 'step_result':
        time.value = msg.time
        // Re-attach buffers to reactive refs
        positions.value = msg.positions
        velocities.value = msg.velocities
        telemetry.value = msg.telemetry
        stepInFlight = false
        if (queuedStepDt > 0) {
          const nextDt = queuedStepDt
          queuedStepDt = 0
          dispatchStep(nextDt, queuedControls ?? defaultControls())
          queuedControls = null
        }
        break

      case 'error':
        logDebug('physics-worker:error', { message: msg.message })
        ready.value = false
        stepInFlight = false
        queuedStepDt = 0
        queuedControls = null
        settleReady(new Error(msg.message))
        break
    }
  }

  function initWorker() {
    if (worker || disposed) return
    // ponytail: Vite worker import — add bundler config if worker path changes
    worker = new Worker(
      new URL('../workers/physics.worker.ts', import.meta.url),
      { type: 'module' },
    )
    worker.onmessage = handleMessage
    worker.onerror = (e) => {
      const error = new Error(e.message || 'Physics worker failed')
      logDebug('physics-worker:error', { message: error.message })
      ready.value = false
      stepInFlight = false
      queuedStepDt = 0
      settleReady(error)
    }
  }

  function sendMessage(msg: PhysicsInMessage, transfer?: Transferable[]) {
    if (!worker || disposed) return
    worker.postMessage(msg, transfer ?? [])
  }

  function waitForReady(): Promise<void> {
    if (ready.value) return Promise.resolve()
    return new Promise<void>((resolve, reject) => {
      pendingResolves.push(resolve)
      pendingRejects.push(reject)
    })
  }

  async function init() {
    initWorker()
    // ponytail: hardcoded WASM path — add config/env lookup when mod pipeline exists
    sendMessage({ type: 'init', wasmUrl: '/pkg/kemudi_physics.js' })
    await waitForReady()
  }

  async function loadVehicle(vehicle: VehicleDefinition, terrainProfile: TerrainProfileId = 0, opts?: { groundFriction?: number; surfaceRoughness?: number; surfaceMoisture?: number; surfaceCompactness?: number; surfacePresetIndex?: number }) {
    if (disposed) throw new Error('Physics engine is disposed')
    if (!worker) await init()
    ready.value = false
    lastVehicle = vehicle
    lastTerrainProfile = terrainProfile
    sendMessage({
      type: 'load_vehicle',
      vehicle,
      terrainProfile,
      groundFriction: opts?.groundFriction,
      surfaceRoughness: opts?.surfaceRoughness,
      surfaceMoisture: opts?.surfaceMoisture,
      surfaceCompactness: opts?.surfaceCompactness,
      surfacePresetIndex: opts?.surfacePresetIndex,
    })
    await waitForReady()
  }

  async function restart() {
    if (disposed) throw new Error('Physics engine is disposed')
    if (worker) {
      worker.terminate()
      worker = null
    }
    ready.value = false
    stepInFlight = false
    queuedStepDt = 0
    queuedControls = null
    await init()
    if (lastVehicle) await loadVehicle(lastVehicle, lastTerrainProfile)
  }

  function step(dt: number, controls: PhysicsControls = defaultControls()) {
    if (!worker || !ready.value || disposed) return
    if (!Number.isFinite(dt) || dt <= 0) return
    if (stepInFlight) {
      queuedStepDt = Math.min(queuedStepDt + dt, 0.25)
      queuedControls = controls
      return
    }
    dispatchStep(Math.min(dt, 0.25), controls)
  }

  function applyForce(nodeId: number, fx: number, fy: number, fz: number) {
    sendMessage({ type: 'apply_force', nodeId, fx, fy, fz })
  }

  function setTransmissionMode(mode: 'manual' | 'automatic') {
    sendMessage({ type: 'set_transmission_mode', mode: mode === 'automatic' ? 1 : 0 })
  }

  function setTcmFault(faultId: number, active: boolean, intermittent = false, seed?: number) {
    if (!Number.isInteger(faultId) || faultId < 0 || faultId > 11) return
    sendMessage({ type: 'set_tcm_fault', faultId, active, intermittent, seed })
  }

  function clearTcmFaults() {
    sendMessage({ type: 'clear_tcm_faults' })
  }

  function reset() {
    ready.value = false
    stepInFlight = false
    queuedStepDt = 0
    queuedControls = null
    sendMessage({ type: 'reset' })
    positions.value = new Float64Array(0)
    velocities.value = new Float64Array(0)
    telemetry.value = new Float64Array(TELEMETRY_LENGTH)
    time.value = 0
  }

  function dispose() {
    if (disposed) return
    disposed = true
    if (worker) {
      worker.terminate()
      worker = null
    }
    settleReady(new Error('Physics engine disposed'))
    logDebug('physics-worker:error', { message: 'disposed' })
  }

  return {
    ready: readonly(ready),
    positions,
    velocities,
    telemetry,
    time: readonly(time),
    init,
    loadVehicle,
    setTransmissionMode,
    setTcmFault,
    clearTcmFaults,
    restart,
    step,
    applyForce,
    reset,
    dispose,
  }
}

function defaultControls(): PhysicsControls {
  return { steering: 0, throttle: 0, brake: 0, clutch: 0, handbrake: false, gearUp: false, gearDown: false, engineOn: false }
}
