/**
 * Physics Web Worker — loads WASM and runs the simulation off-main-thread.
 *
 * Communication uses structured clone; positions/velocities are transferred
 * as Float64Array (zero-copy) back to the main thread.
 */
import type { PhysicsInMessage, PhysicsOutMessage } from '~/types/physics'

interface WasmWorld {
  add_node(id: number, x: number, y: number, z: number, mass: number, fixed: boolean): void
  add_beam(id: number, nodeA: number, nodeB: number, stiffness: number, damping: number, strength: number): void
  step(dt: number): void
  apply_force(nodeId: number, fx: number, fy: number, fz: number): void
  get_positions_flat(): Float64Array
  get_velocities_flat(): Float64Array
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
        wasmModule = await import(msg.wasmUrl) as unknown as WasmModule
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
        })
        for (const b of vehicle.beams) {
          const nodeA = nodeIndex.get(b.nodeA)
          const nodeB = nodeIndex.get(b.nodeB)
          if (nodeA === undefined || nodeB === undefined) {
            throw new Error(`Beam ${b.id} references an unknown node`)
          }
          world.add_beam(b.id, nodeA, nodeB, b.stiffness, b.damping, b.strength)
        }
        send({ type: 'ready' })
        break
      }

      case 'step': {
        if (!world) throw new Error('Physics not initialized')
        world.step(msg.dt)
        const positions = world.get_positions_flat()
        const velocities = world.get_velocities_flat()
        // Transfer ownership back — zero copy
        const out: PhysicsOutMessage = {
          type: 'step_result',
          time: world.get_time(),
          positions,
          velocities,
        }
        ctx.postMessage(out, [positions.buffer, velocities.buffer])
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

function send(msg: PhysicsOutMessage) {
  ctx.postMessage(msg)
}
