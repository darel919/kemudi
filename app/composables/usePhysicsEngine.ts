import { ref, shallowRef, readonly } from 'vue'
import type {
  PhysicsInMessage,
  PhysicsOutMessage,
  VehicleDefinition,
} from '~/types/physics'
import { logDebug } from '~/utils/debug'

export interface PhysicsEngineHandle {
  /** Whether the WASM worker is ready to accept steps */
  ready: ReturnType<typeof readonly>
  /** Current node positions [x0,y0,z0, x1,y1,z1, ...] */
  positions: ReturnType<typeof shallowRef<Float64Array>>
  /** Current node velocities [vx0,vy0,vz0, ...] */
  velocities: ReturnType<typeof shallowRef<Float64Array>>
  /** Current simulation time in seconds */
  time: ReturnType<typeof readonly>
  /** Initialize the WASM physics engine */
  init(): Promise<void>
  /** Load a vehicle definition into the engine */
  loadVehicle(vehicle: VehicleDefinition): Promise<void>
  /** Step the simulation forward by dt seconds */
  step(dt: number): void
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

  let worker: Worker | null = null
  const pendingResolves: Array<() => void> = []
  const pendingRejects: Array<(err: Error) => void> = []
  let stepInFlight = false
  let queuedStepDt = 0
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

  function dispatchStep(dt: number) {
    if (!worker || disposed || !ready.value || stepInFlight) return
    stepInFlight = true
    sendMessage({ type: 'step', dt })
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
        stepInFlight = false
        if (queuedStepDt > 0) {
          const nextDt = queuedStepDt
          queuedStepDt = 0
          dispatchStep(nextDt)
        }
        break

      case 'error':
        logDebug('physics-worker:error', { message: msg.message })
        ready.value = false
        stepInFlight = false
        queuedStepDt = 0
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

  async function loadVehicle(vehicle: VehicleDefinition) {
    if (disposed) throw new Error('Physics engine is disposed')
    if (!worker) await init()
    ready.value = false
    sendMessage({ type: 'load_vehicle', vehicle })
    await waitForReady()
  }

  function step(dt: number) {
    if (!worker || !ready.value || disposed) return
    if (!Number.isFinite(dt) || dt <= 0) return
    if (stepInFlight) {
      queuedStepDt = Math.min(queuedStepDt + dt, 0.25)
      return
    }
    dispatchStep(Math.min(dt, 0.25))
  }

  function applyForce(nodeId: number, fx: number, fy: number, fz: number) {
    sendMessage({ type: 'apply_force', nodeId, fx, fy, fz })
  }

  function reset() {
    ready.value = false
    stepInFlight = false
    queuedStepDt = 0
    sendMessage({ type: 'reset' })
    positions.value = new Float64Array(0)
    velocities.value = new Float64Array(0)
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
    time: readonly(time),
    init,
    loadVehicle,
    step,
    applyForce,
    reset,
    dispose,
  }
}
