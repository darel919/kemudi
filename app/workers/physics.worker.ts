/**
 * Physics Web Worker — loads WASM and runs the simulation off-main-thread.
 *
 * Communication uses structured clone; positions/velocities are transferred
 * as Float64Array (zero-copy) back to the main thread.
 */
import type { PhysicsInMessage, PhysicsOutMessage, TerrainProfileId, VehicleDefinition } from '~/types/physics'

interface WasmWorld {
  add_node(id: number, x: number, y: number, z: number, mass: number, fixed: boolean): void
  set_node_collision(nodeId: number, collision: boolean): void
  add_beam(id: number, nodeA: number, nodeB: number, stiffness: number, damping: number, strength: number): void
  add_triangle(a: number, b: number, c: number): void
  configure_runtime(
    idleRpm: number, redlineRpm: number, limiterRpm: number, throttleResponse: number, engineBraking: number,
    torqueRpms: Float64Array, torqueValues: Float64Array, gearRatios: Float64Array, finalDrive: number,
    reverseRatio: number, transmissionMode: number, shiftDelay: number, differentialMode: number,
    differentialBias: number, springRates: Float64Array, dampings: Float64Array, reboundDampings: Float64Array,
    restLengths: Float64Array, travels: Float64Array, radii: Float64Array, tireCompounds: Uint8Array,
    tirePressures: Float64Array, fuelCapacity: number, fuelConsumption: number, idleConsumption: number,
    absEnabled: boolean, tractionControlEnabled: boolean, vscEnabled: boolean,
    adasForwardCollisionWarning: boolean, adasAutomaticEmergencyBraking: boolean,
  ): void
  set_terrain_profile(profile: TerrainProfileId): void
  set_transmission_mode(mode: number): void
  set_automatic_shift_schedule(upshiftRpm: number, downshiftRpm: number): void
  set_adas_target(distance: number, relativeSpeed: number): void
  set_controls(steering: number, throttle: number, brake: number, clutch: number, handbrake: boolean, gearUp: boolean, gearDown: boolean, engineOn: boolean): void
  step(dt: number): void
  apply_force(nodeId: number, fx: number, fy: number, fz: number): void
  get_positions_flat(): Float64Array
  get_velocities_flat(): Float64Array
  get_telemetry_flat(): Float64Array
  get_time(): number
  free(): void
}

interface WasmModule {
  default(): Promise<unknown>
  PhysicsWorld: new () => WasmWorld
}

let wasmModule: WasmModule | null = null
let world: WasmWorld | null = null

const ctx = self as unknown as Worker

ctx.onmessage = async (e: MessageEvent<PhysicsInMessage>) => {
  const msg = e.data

  try {
    switch (msg.type) {
      case 'init': {
        // This is a public runtime asset selected by the physics loader. Its
        // URL is intentionally resolved at runtime and cannot be statically
        // analysed by Vite.
        wasmModule = await import(/* @vite-ignore */ msg.wasmUrl) as unknown as WasmModule
        await wasmModule.default()
        world = new wasmModule.PhysicsWorld()
        send({ type: 'ready' })
        break
      }

      case 'load_vehicle': {
        if (!world) throw new Error('Physics not initialized')
        const { vehicle } = msg
        world.free()
        world = new wasmModule!.PhysicsWorld()
        const nodeIndex = new Map<number, number>()
        vehicle.nodes.forEach((n, index) => {
          nodeIndex.set(n.id, index)
          world!.add_node(index, n.x, n.y, n.z, n.mass, n.fixed)
          world!.set_node_collision(index, n.collision ?? index >= 4)
        })
        for (const b of vehicle.beams) {
          const nodeA = nodeIndex.get(b.nodeA)
          const nodeB = nodeIndex.get(b.nodeB)
          if (nodeA === undefined || nodeB === undefined) {
            throw new Error(`Beam ${b.id} references an unknown node`)
          }
          world.add_beam(b.id, nodeA, nodeB, b.stiffness, b.damping, b.strength)
        }
        for (const triangle of vehicle.triangles ?? []) {
          const a = nodeIndex.get(triangle[0])
          const b = nodeIndex.get(triangle[1])
          const c = nodeIndex.get(triangle[2])
          if (a !== undefined && b !== undefined && c !== undefined) world.add_triangle(a, b, c)
        }
        configureRuntime(world, vehicle)
        world.set_automatic_shift_schedule(
          vehicle.transmission?.autoShiftUpRpm ?? 5000,
          vehicle.transmission?.autoShiftDownRpm ?? 2200,
        )
        world.set_terrain_profile(msg.terrainProfile)
        send({ type: 'ready' })
        break
      }

      case 'set_transmission_mode': {
        if (!world) throw new Error('Physics not initialized')
        world.set_transmission_mode(msg.mode)
        break
      }

      case 'step': {
        if (!world) throw new Error('Physics not initialized')
        world.set_controls(
          msg.controls.steering,
          msg.controls.throttle,
          msg.controls.brake,
          msg.controls.clutch,
          msg.controls.handbrake,
          msg.controls.gearUp,
          msg.controls.gearDown,
          msg.controls.engineOn,
        )
        world.set_adas_target(
          msg.controls.adasTargetDistance ?? 0,
          msg.controls.adasTargetRelativeSpeed ?? 0,
        )
        world.step(msg.dt)
        const positions = world.get_positions_flat()
        const velocities = world.get_velocities_flat()
        const telemetry = world.get_telemetry_flat()
        // Transfer ownership back — zero copy
        const out: PhysicsOutMessage = {
          type: 'step_result',
          time: world.get_time(),
          positions,
          velocities,
          telemetry,
        }
        ctx.postMessage(out, [positions.buffer, velocities.buffer, telemetry.buffer])
        break
      }

      case 'apply_force': {
        if (!world) throw new Error('Physics not initialized')
        world.apply_force(msg.nodeId, msg.fx, msg.fy, msg.fz)
        break
      }

      case 'reset': {
        if (world) {
          world.free()
          world = null
        }
        send({ type: 'ready' })
        break
      }
    }
  } catch (err) {
    send({ type: 'error', message: err instanceof Error ? err.message : String(err) })
  }
}

function configureRuntime(runtime: WasmWorld, vehicle: VehicleDefinition) {
  const engine = vehicle.engine
  const transmission = vehicle.transmission
  const suspension = vehicle.suspension
  const tires = vehicle.tires
  const fuel = vehicle.fuel
  const torqueCurve = engine?.torqueCurve ?? [[0, 100], [1000, 150], [3000, 250], [7000, 160]] as [number, number][]
  const wheelConfigs = suspension?.wheels ?? Array.from({ length: 4 }, () => ({ springRate: 30000, damping: 4000, reboundDamping: 2500, restLength: 0.35, travel: 0.2, tireRadius: 0.33 }))
  const tireConfigs = tires ?? Array.from({ length: 4 }, () => ({ compound: 'street' as const, nominalPressure: 32 }))
  const diffType = transmission?.differential.type ?? 'open'
  runtime.configure_runtime(
    engine?.idleRpm ?? 800,
    engine?.redlineRpm ?? 7000,
    engine?.revLimiterRpm ?? 7200,
    engine?.throttleResponse ?? 0.8,
    engine?.engineBraking ?? 30,
    Float64Array.from(torqueCurve, point => point[0]),
    Float64Array.from(torqueCurve, point => point[1]),
    Float64Array.from(transmission?.gearRatios ?? [3.5, 2.1, 1.4, 1, 0.7]),
    transmission?.finalDrive ?? 3.7,
    transmission?.reverseRatio ?? -3.2,
    transmission?.mode === 'automatic' ? 1 : 0,
    transmission?.shiftDelay ?? 0.15,
    diffType === 'locked' ? 1 : diffType === 'limited_slip' ? 2 : 0,
    transmission?.differential.bias ?? 0.5,
    Float64Array.from(wheelConfigs, wheel => wheel.springRate),
    Float64Array.from(wheelConfigs, wheel => wheel.damping),
    Float64Array.from(wheelConfigs, wheel => wheel.reboundDamping),
    Float64Array.from(wheelConfigs, wheel => wheel.restLength),
    Float64Array.from(wheelConfigs, wheel => wheel.travel),
    Float64Array.from(wheelConfigs, wheel => wheel.tireRadius),
    Uint8Array.from(tireConfigs, tire => ({ sport: 0, street: 1, offroad: 2, mud: 3, snow: 4 }[tire.compound] ?? 1)),
    Float64Array.from(tireConfigs, tire => tire.nominalPressure),
    fuel?.capacity ?? 60,
    fuel?.consumptionRate ?? 0.01,
    fuel?.idleConsumptionRate ?? 0.0005,
    vehicle.safety?.abs ?? false,
    vehicle.safety?.tractionControl ?? false,
    vehicle.safety?.vsc ?? false,
    vehicle.safety?.adasForwardCollisionWarning ?? false,
    vehicle.safety?.adasAutomaticEmergencyBraking ?? false,
  )
}

function send(msg: PhysicsOutMessage) {
  ctx.postMessage(msg)
}
