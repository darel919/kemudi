/** Message types for physics worker communication */

// Main thread → Worker
export type PhysicsInMessage =
  | { type: 'init'; wasmUrl: string }
  | { type: 'load_vehicle'; vehicle: VehicleDefinition }
  | { type: 'step'; dt: number }
  | { type: 'apply_force'; nodeId: number; fx: number; fy: number; fz: number }
  | { type: 'reset' }

// Worker → Main thread
export type PhysicsOutMessage =
  | { type: 'ready' }
  | { type: 'step_result'; time: number; positions: Float64Array; velocities: Float64Array }
  | { type: 'error'; message: string }

export interface VehicleNodeDef {
  id: number
  x: number
  y: number
  z: number
  mass: number
  fixed: boolean
}

export interface VehicleBeamDef {
  id: number
  nodeA: number
  nodeB: number
  stiffness: number
  damping: number
  strength: number
}

export interface VehicleDefinition {
  nodes: VehicleNodeDef[]
  beams: VehicleBeamDef[]
}
