import { TELEMETRY_ADAS } from '~/types/physics'

export interface AdasTelemetryState {
  fcw: boolean
  aeb: boolean
  confidence: number
  targetDetected: boolean
  targetDistance: number
  timeToCollision: number
  relativeSpeed: number
  brakeCommand: number
  sensorFaults: number
}

export function readAdasTelemetry(snapshot: ArrayLike<number>): AdasTelemetryState {
  const valueAt = (index: number, fallback = 0) => {
    const value = snapshot[index]
    return typeof value === 'number' && Number.isFinite(value) ? value : fallback
  }
  const targetDetected = valueAt(TELEMETRY_ADAS.targetDetected) > 0.5
  const timeToCollision = Math.max(0, valueAt(TELEMETRY_ADAS.timeToCollision, 30))

  return {
    fcw: valueAt(TELEMETRY_ADAS.fcw) > 0.5,
    aeb: valueAt(TELEMETRY_ADAS.aeb) > 0.5,
    confidence: Math.max(0, Math.min(1, valueAt(TELEMETRY_ADAS.confidence))),
    targetDetected,
    targetDistance: Math.max(0, valueAt(TELEMETRY_ADAS.targetDistance)),
    timeToCollision: targetDetected ? timeToCollision : 30,
    relativeSpeed: valueAt(TELEMETRY_ADAS.relativeSpeed),
    brakeCommand: Math.max(0, Math.min(1, valueAt(TELEMETRY_ADAS.brakeCommand))),
    sensorFaults: Math.max(0, valueAt(TELEMETRY_ADAS.sensorFaults)),
  }
}

export function adasTelemetryUpdates(state: AdasTelemetryState): [string, number][] {
  return [
    ['safety.adas_forward_collision_warning', state.fcw ? 1 : 0],
    ['safety.adas_aeb_active', state.aeb ? 1 : 0],
    ['safety.adas_sensor_confidence', state.confidence * 100],
    ['safety.adas_target_detected', state.targetDetected ? 1 : 0],
    ['safety.adas_target_distance', state.targetDistance],
    ['safety.adas_time_to_collision', state.timeToCollision],
    ['safety.adas_target_relative_speed', state.relativeSpeed],
    ['safety.adas_brake_command', state.brakeCommand * 100],
    ['safety.adas_sensor_faults', state.sensorFaults],
  ]
}
