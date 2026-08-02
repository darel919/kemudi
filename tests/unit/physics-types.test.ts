import { describe, it, expect } from 'vitest'
import type {
  VehicleNodeDef,
  VehicleBeamDef,
  VehicleDefinition,
  PhysicsInMessage,
  PhysicsOutMessage,
  MapPhysicsData,
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
      massProperties: {
        totalMass: 2,
        centerOfMass: { x: 0, y: 0, z: 1 },
        inertia: { x: 1, y: 2, z: 3 },
      },
      weightDistribution: { front: 0.5, rear: 0.5 },
      drivetrain: { layout: 'awd' },
      aerodynamics: {
        frontalArea: 1.8,
        dragCoefficient: 0.32,
        liftCoefficient: 0,
        centerOfPressure: { x: 0, y: 0.5, z: 0.8 },
      },
    }
    expect(vehicle.nodes).toHaveLength(2)
    expect(vehicle.beams).toHaveLength(1)
    expect(vehicle.drivetrain.layout).toBe('awd')
    expect(vehicle.massProperties.totalMass).toBe(2)
  })

  it('PhysicsInMessage types are correct', () => {
    const init: PhysicsInMessage = { type: 'init', wasmUrl: '/pkg/test.js' }
    const step: PhysicsInMessage = { type: 'step', dt: 1 / 60, controls: { steering: 0, throttle: 0, brake: 0, clutch: 0, handbrake: false, gearUp: false, gearDown: false, engineOn: false } }
    const force: PhysicsInMessage = { type: 'apply_force', nodeId: 0, fx: 1, fy: 2, fz: 3 }
    const load: PhysicsInMessage = {
      type: 'load_vehicle',
      vehicle: { nodes: [], beams: [] },
      terrainProfile: 0,
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
      telemetry: new Float64Array(32),
    }
    const error: PhysicsOutMessage = { type: 'error', message: 'fail' }
    expect(ready.type).toBe('ready')
    expect(result.type).toBe('step_result')
    expect(result.positions.length).toBe(3)
    expect(error.type).toBe('error')
  })

  it('carries bounded map collision and road surface data across the worker boundary', () => {
    const map: MapPhysicsData = {
      primitives: [{ kind: 'sphere', center: { x: 0, y: 1, z: 2 }, radius: 2, restitution: 0.2, friction: 0.6 }],
      boundaries: [{ center: { x: 0, y: 1, z: -10 }, halfExtents: { x: 5, y: 1, z: 0.5 }, restitution: 0.1, friction: 0.8 }],
      roads: [{ name: 'road', width: 5, surfaceId: 2, friction: 0.7, roughness: 0.4, moisture: 0.1, compactness: 0.9, points: [{ x: 0, z: 0 }, { x: 0, z: 10 }] }],
    }
    const load: PhysicsInMessage = { type: 'load_vehicle', vehicle: { nodes: [], beams: [] }, terrainProfile: 0, map }
    expect(load.map?.roads[0]?.surfaceId).toBe(2)
  })
})
