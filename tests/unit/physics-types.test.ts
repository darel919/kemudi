import { describe, it, expect } from 'vitest'
import type {
  VehicleNodeDef,
  VehicleBeamDef,
  VehicleDefinition,
  PhysicsInMessage,
  PhysicsOutMessage,
} from '../../app/types/physics'

describe('physics types', () => {
  it('VehicleNodeDef has required fields', () => {
    const node: VehicleNodeDef = {
      id: 0,
      x: 0, y: 1, z: 0,
      mass: 5.0,
      fixed: false,
    }
    expect(node.id).toBe(0)
    expect(node.fixed).toBe(false)
    expect(node.mass).toBeGreaterThan(0)
  })

  it('VehicleBeamDef has required fields', () => {
    const beam: VehicleBeamDef = {
      id: 0,
      nodeA: 0,
      nodeB: 1,
      stiffness: 1000,
      damping: 0.5,
      strength: 500,
    }
    expect(beam.nodeA).not.toBe(beam.nodeB)
    expect(beam.stiffness).toBeGreaterThan(0)
  })

  it('VehicleDefinition assembles nodes and beams', () => {
    const vehicle: VehicleDefinition = {
      nodes: [
        { id: 0, x: 0, y: 0, z: 0, mass: 1, fixed: true },
        { id: 1, x: 0, y: 0, z: 2, mass: 1, fixed: false },
      ],
      beams: [
        { id: 0, nodeA: 0, nodeB: 1, stiffness: 1000, damping: 0.5, strength: 500 },
      ],
    }
    expect(vehicle.nodes).toHaveLength(2)
    expect(vehicle.beams).toHaveLength(1)
  })

  it('PhysicsInMessage types are correct', () => {
    const init: PhysicsInMessage = { type: 'init', wasmUrl: '/pkg/test.js' }
    const step: PhysicsInMessage = { type: 'step', dt: 1 / 60 }
    const force: PhysicsInMessage = { type: 'apply_force', nodeId: 0, fx: 1, fy: 2, fz: 3 }
    const load: PhysicsInMessage = {
      type: 'load_vehicle',
      vehicle: { nodes: [], beams: [] },
    }
    const reset: PhysicsInMessage = { type: 'reset' }
    expect(init.type).toBe('init')
    expect(step.type).toBe('step')
    expect(force.type).toBe('apply_force')
    expect(load.type).toBe('load_vehicle')
    expect(reset.type).toBe('reset')
  })

  it('PhysicsOutMessage types are correct', () => {
    const ready: PhysicsOutMessage = { type: 'ready' }
    const result: PhysicsOutMessage = {
      type: 'step_result',
      time: 1.0,
      positions: new Float64Array([0, 1, 2]),
      velocities: new Float64Array([0, 0, 0]),
    }
    const error: PhysicsOutMessage = { type: 'error', message: 'fail' }
    expect(ready.type).toBe('ready')
    expect(result.type).toBe('step_result')
    expect(result.positions.length).toBe(3)
    expect(error.type).toBe('error')
  })
})
