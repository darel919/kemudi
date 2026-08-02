import { describe, it, expect, beforeEach } from 'vitest'
import {
  VIRTUAL_OBD_PIDS,
  TELEMETRY_SCHEMA_VERSION,
  type SubsystemId,
  type TelemetrySignalDefinition,
} from '../../app/types/telemetry'
import { useTelemetryStore } from '../../app/stores/telemetry'
import { setActivePinia, createPinia } from 'pinia'

// ---------------------------------------------------------------------------
// VIRTUAL_OBD_PIDS validation
// ---------------------------------------------------------------------------

describe('VIRTUAL_OBD_PIDS', () => {
  const pids = VIRTUAL_OBD_PIDS as TelemetrySignalDefinition[]

  it('has at least 60 signals', () => {
    expect(pids.length).toBeGreaterThanOrEqual(60)
  })

  it('each signal has all required fields', () => {
    for (const pid of pids) {
      expect(pid.id).toBeTruthy()
      expect(pid.label).toBeTruthy()
      expect(pid.unit).toBeDefined()
      expect(pid.subsystem).toBeTruthy()
      expect(pid.category).toBeTruthy()
      expect(pid.description).toBeTruthy()
    }
  })

  it('signal IDs are unique', () => {
    const ids = pids.map(p => p.id)
    expect(new Set(ids).size).toBe(ids.length)
  })

  it('all subsystem IDs are valid', () => {
    const valid: SubsystemId[] = [
      'engine', 'drivetrain', 'suspension', 'tires',
      'body', 'fuel', 'electrical', 'vehicle', 'safety',
    ]
    for (const pid of pids) {
      expect(valid).toContain(pid.subsystem)
    }
  })

  it('signal IDs follow subsystem.metric naming convention', () => {
    const pattern = /^[a-z]+\.[a-z][a-z0-9_]*$/
    for (const pid of pids) {
      expect(pid.id).toMatch(pattern)
    }
  })

  it('warning thresholds are within min/max bounds', () => {
    for (const pid of pids) {
      if (pid.warningThreshold !== undefined) {
        if (pid.minValue !== undefined) {
          expect(pid.warningThreshold).toBeGreaterThanOrEqual(pid.minValue)
        }
        if (pid.maxValue !== undefined) {
          expect(pid.warningThreshold).toBeLessThanOrEqual(pid.maxValue)
        }
      }
      if (pid.faultThreshold !== undefined) {
        if (pid.minValue !== undefined) {
          expect(pid.faultThreshold).toBeGreaterThanOrEqual(pid.minValue)
        }
        if (pid.maxValue !== undefined) {
          expect(pid.faultThreshold).toBeLessThanOrEqual(pid.maxValue)
        }
      }
    }
  })

  it('schema version is a positive integer', () => {
    expect(TELEMETRY_SCHEMA_VERSION).toBeGreaterThanOrEqual(1)
  })

  it('engine stress signal IDs exist with valid units', () => {
    const byId = new Map(pids.map(p => [p.id, p]))
    const stressIds = [
      'engine.stress_thermal', 'engine.stress_oil',
      'engine.stress_overrev', 'engine.stress_lugging',
      'engine.derate_factor', 'engine.blowup_state',
    ]
    for (const id of stressIds) {
      const sig = byId.get(id)
      expect(sig).toBeDefined()
      expect(sig!.unit).toBeDefined()
      expect(typeof sig!.unit).toBe('string')
    }
  })

  it('safety signal IDs exist with valid units', () => {
    const byId = new Map(pids.map(p => [p.id, p]))
    const safetyIds = [
      'safety.abs_wheel_speed_fl', 'safety.abs_wheel_speed_fr',
      'safety.abs_wheel_speed_rl', 'safety.abs_wheel_speed_rr',
      'safety.abs_active', 'safety.abs_brake_pressure_mod', 'safety.abs_mode',
      'safety.tc_active', 'safety.tc_throttle_mod', 'safety.tc_mode',
      'safety.vsc_yaw_error', 'safety.vsc_correction_active',
      'safety.vsc_brake_torque_l', 'safety.vsc_brake_torque_r',
      'safety.vsc_torque_reduction', 'safety.vsc_mode',
      'safety.adas_time_to_collision', 'safety.adas_target_detected',
      'safety.adas_target_distance', 'safety.adas_aeb_active',
      'safety.adas_acc_target_speed', 'safety.adas_ldw_warning',
      'safety.adas_sensor_faults',
    ]
    for (const id of safetyIds) {
      const sig = byId.get(id)
      expect(sig).toBeDefined()
      expect(sig!.unit).toBeDefined()
      expect(typeof sig!.unit).toBe('string')
      expect(sig!.subsystem).toBe('safety')
    }
  })

  it('no duplicate IDs across subsystems', () => {
    const seen = new Set<string>()
    for (const pid of pids) {
      expect(seen.has(pid.id)).toBe(false)
      seen.add(pid.id)
    }
  })
})

// ---------------------------------------------------------------------------
// Store integration
// ---------------------------------------------------------------------------

describe('telemetry store', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('registerSignal + updateSignal + getHistory flow', () => {
    const store = useTelemetryStore()

    store.registerSignal({
      id: 'engine.rpm',
      label: 'Engine RPM',
      value: 0,
      unit: 'rpm',
      warning: false,
      fault: false,
      timestamp: 0,
      quality: 'good',
    })

    store.updateSignal('engine.rpm', 3000)
    const sig = store.getSignal('engine.rpm')
    expect(sig).toBeDefined()
    expect(sig!.value).toBe(3000)
    expect(sig!.quality).toBe('good')

    store.updateSignal('engine.rpm', 4500)
    const history = store.getHistory('engine.rpm')
    expect(history.length).toBe(2)
    expect(history[0].value).toBe(3000)
    expect(history[1].value).toBe(4500)
  })

  it('registerAllSignals registers all PIDs without duplicates', () => {
    const store = useTelemetryStore()
    store.registerAllSignals(VIRTUAL_OBD_PIDS)
    expect(store.allSignals.length).toBe(VIRTUAL_OBD_PIDS.length)

    // Calling again should not duplicate
    store.registerAllSignals(VIRTUAL_OBD_PIDS)
    expect(store.allSignals.length).toBe(VIRTUAL_OBD_PIDS.length)
  })

  it('signal freshness detection: >2s old is stale', () => {
    const store = useTelemetryStore()
    const oldTime = Date.now() - 3000

    store.registerSignal({
      id: 'engine.rpm',
      label: 'Engine RPM',
      value: 0,
      unit: 'rpm',
      warning: false,
      fault: false,
      timestamp: 0,
      quality: 'good',
    })
    // Manually set stale timestamp
    store.signals.set('engine.rpm', {
      id: 'engine.rpm',
      label: 'Engine RPM',
      value: 1000,
      unit: 'rpm',
      warning: false,
      fault: false,
      timestamp: oldTime,
      quality: 'good',
    })

    const sig = store.getSignal('engine.rpm')!
    const isStale = Date.now() - sig.timestamp > 2000
    expect(isStale).toBe(true)
  })

  it('signal freshness: fresh signal not stale', () => {
    const store = useTelemetryStore()
    store.registerSignal({
      id: 'engine.rpm',
      label: 'Engine RPM',
      value: 0,
      unit: 'rpm',
      warning: false,
      fault: false,
      timestamp: 0,
      quality: 'good',
    })
    store.updateSignal('engine.rpm', 3000)

    const sig = store.getSignal('engine.rpm')!
    const isStale = Date.now() - sig.timestamp > 2000
    expect(isStale).toBe(false)
  })

  it('stale signal display logic: quality field can be set to stale', () => {
    const store = useTelemetryStore()
    store.registerSignal({
      id: 'engine.rpm',
      label: 'Engine RPM',
      value: 0,
      unit: 'rpm',
      warning: false,
      fault: false,
      timestamp: 0,
      quality: 'good',
    })
    store.updateSignal('engine.rpm', 3000, 'stale')
    const sig = store.getSignal('engine.rpm')!
    expect(sig.quality).toBe('stale')
  })

  it('store reset clears all signals and history', () => {
    const store = useTelemetryStore()
    store.registerSignal({
      id: 'engine.rpm',
      label: 'Engine RPM',
      value: 0,
      unit: 'rpm',
      warning: false,
      fault: false,
      timestamp: 0,
      quality: 'good',
    })
    store.updateSignal('engine.rpm', 3000)
    expect(store.allSignals.length).toBe(1)
    expect(store.getHistory('engine.rpm').length).toBe(1)

    store.$reset()
    expect(store.allSignals.length).toBe(0)
    expect(store.getHistory('engine.rpm').length).toBe(0)
  })

  it('setWarning and setFault toggle correctly', () => {
    const store = useTelemetryStore()
    store.registerSignal({
      id: 'engine.coolant_temp',
      label: 'Coolant Temp',
      value: 0,
      unit: '°C',
      warning: false,
      fault: false,
      timestamp: 0,
      quality: 'good',
    })

    store.setWarning('engine.coolant_temp', true)
    expect(store.getSignal('engine.coolant_temp')!.warning).toBe(true)

    store.setFault('engine.coolant_temp', true)
    expect(store.getSignal('engine.coolant_temp')!.fault).toBe(true)

    expect(store.hasWarnings).toBe(true)
  })
})
