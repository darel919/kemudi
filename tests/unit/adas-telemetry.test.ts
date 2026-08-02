import { describe, expect, it } from 'vitest'
import { adasTelemetryUpdates, readAdasTelemetry } from '~/utils/adasTelemetry'
import { TELEMETRY_ADAS, TELEMETRY_LENGTH } from '~/types/physics'

describe('ADAS telemetry mapping', () => {
  it('maps the worker snapshot into MFD values and intervention state', () => {
    const snapshot = new Float64Array(TELEMETRY_LENGTH)
    snapshot[TELEMETRY_ADAS.fcw] = 1
    snapshot[TELEMETRY_ADAS.aeb] = 1
    snapshot[TELEMETRY_ADAS.confidence] = 0.8
    snapshot[TELEMETRY_ADAS.targetDetected] = 1
    snapshot[TELEMETRY_ADAS.targetDistance] = 4.5
    snapshot[TELEMETRY_ADAS.timeToCollision] = 0.9
    snapshot[TELEMETRY_ADAS.relativeSpeed] = -10
    snapshot[TELEMETRY_ADAS.brakeCommand] = 0.72

    const state = readAdasTelemetry(snapshot)
    expect(state).toEqual({
      fcw: true,
      aeb: true,
      confidence: 0.8,
      targetDetected: true,
      targetDistance: 4.5,
      timeToCollision: 0.9,
      relativeSpeed: -10,
      brakeCommand: 0.72,
      sensorFaults: 0,
    })

    const updates = new Map(adasTelemetryUpdates(state))
    expect(updates.get('safety.adas_forward_collision_warning')).toBe(1)
    expect(updates.get('safety.adas_aeb_active')).toBe(1)
    expect(updates.get('safety.adas_sensor_confidence')).toBe(80)
    expect(updates.get('safety.adas_time_to_collision')).toBe(0.9)
    expect(updates.get('safety.adas_brake_command')).toBe(72)
  })

  it('does not report a stale TTC when there is no target', () => {
    const state = readAdasTelemetry(new Float64Array(TELEMETRY_LENGTH))
    expect(state.targetDetected).toBe(false)
    expect(state.timeToCollision).toBe(30)
    expect(state.brakeCommand).toBe(0)
  })
})
