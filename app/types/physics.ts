/** Message types for physics worker communication */

// Main thread → Worker
export type PhysicsInMessage =
  | { type: 'init'; wasmUrl: string }
  | { type: 'load_vehicle'; vehicle: VehicleDefinition; terrainProfile: TerrainProfileId }
  | { type: 'set_transmission_mode'; mode: 0 | 1 }
  | { type: 'set_tcm_fault'; faultId: number; active: boolean; intermittent?: boolean; seed?: number }
  | { type: 'clear_tcm_faults' }
  | { type: 'step'; dt: number; controls: PhysicsControls }
  | { type: 'apply_force'; nodeId: number; fx: number; fy: number; fz: number }
  | { type: 'reset' }

// Worker → Main thread
export type PhysicsOutMessage =
  | { type: 'ready' }
  | { type: 'step_result'; time: number; positions: Float64Array; velocities: Float64Array; telemetry: Float64Array }
  | { type: 'error'; message: string }

export type TerrainProfileId = 0 | 1 | 2

export const TELEMETRY_LENGTH = 80

export interface PhysicsControls {
  steering: number
  throttle: number
  brake: number
  clutch: number
  handbrake: boolean
  gearUp: boolean
  gearDown: boolean
  engineOn: boolean
  adasTargetDistance?: number
  adasTargetRelativeSpeed?: number
}

export interface VehicleNodeDef {
  id: number
  x: number
  y: number
  z: number
  mass: number
  fixed: boolean
  collision?: boolean
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
  triangles?: [number, number, number][]
  engine?: VehicleEngineDefinition
  transmission?: VehicleTransmissionDefinition
  suspension?: VehicleSuspensionDefinition
  tires?: VehicleTireDefinition[]
  fuel?: VehicleFuelDefinition
  safety?: VehicleSafetyDefinition
  body?: VehicleBodyDefinition
}

export interface VehicleBodyDefinition {
  material: string
  crumpleFactor: number
  /** Optional path to a GLTF/GLB body mesh. Falls back to procedural box when absent. */
  bodyMesh?: string
}

export interface VehicleEngineDefinition {
  idleRpm: number
  redlineRpm: number
  revLimiterRpm: number
  throttleResponse: number
  engineBraking: number
  torqueCurve: [number, number][]
}

export interface VehicleTransmissionDefinition {
  mode: 'manual' | 'automatic'
  gearRatios: number[]
  finalDrive: number
  reverseRatio: number
  shiftDelay: number
  autoShiftUpRpm?: number
  autoShiftDownRpm?: number
  differential: { type: 'open' | 'locked' | 'limited_slip'; bias: number }
}

export interface VehicleSuspensionDefinition {
  antiRollBarStiffness?: number
  bumpStopRate?: number
  wheels: Array<{
    springRate: number
    damping: number
    reboundDamping: number
    restLength: number
    travel: number
    tireRadius: number
  }>
}

export interface VehicleTireDefinition {
  compound: 'sport' | 'street' | 'offroad' | 'mud' | 'snow'
  nominalPressure: number
}

export interface VehicleFuelDefinition {
  capacity: number
  consumptionRate: number
  idleConsumptionRate: number
}

export interface VehicleSafetyDefinition {
  abs: boolean
  tractionControl: boolean
  vsc: boolean
  adasForwardCollisionWarning?: boolean
  adasAutomaticEmergencyBraking?: boolean
}
