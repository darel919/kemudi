import type { VehicleDefinition, VehicleNodeDef, VehicleBeamDef } from '~/types/physics'
import { logDebug } from '~/utils/debug'

const STORAGE_KEY = 'kemudi:vehicles'

interface WheelConfig {
  springRate?: number
  damping?: number
  reboundDamping?: number
  restLength?: number
  travel?: number
  unsprungMass?: number
  tireRadius?: number
}

interface VehicleFile {
  version: number
  name: string
  nodes: Array<{
    id: number; x: number; y: number; z: number; mass: number
    fixed?: boolean; collision?: boolean
  }>
  beams: Array<{
    id: number; nodeA: number; nodeB: number
    stiffness?: number; damping?: number; strength?: number
  }>
  engine?: {
    idleRpm?: number; redlineRpm?: number; revLimiterRpm?: number
    displacement?: number; thermalCapacity?: number
    throttleResponse?: number; engineBraking?: number
    torqueCurve?: [number, number][]
    coolingSystem?: { radiatorCapacity?: number; coolantCapacity?: number; fanTemp?: number }
    oilSystem?: { oilCapacity?: number; pumpPressure?: number; pickupPosition?: number }
  }
  transmission?: {
    mode?: string; gearRatios?: number[]; finalDrive?: number; reverseRatio?: number
    clutchEngagement?: number; autoShiftUpRpm?: number; autoShiftDownRpm?: number
    shiftDelay?: number
    differential?: { type?: string; bias?: number }
  }
  suspension?: {
    springRate?: number; damping?: number; travel?: number; antiRollBarStiffness?: number
    wheels?: WheelConfig[]
  }
  tires?: Array<{
    compound?: string; nominalPressure?: number; wearRating?: number
  }>
  brakes?: Array<{ brakeTorqueMax?: number }>
  fuel?: {
    capacity?: number; consumptionRate?: number; fuelDensity?: number; idleConsumptionRate?: number
  }
  damage?: {
    beamYieldStrength?: number
    crumpleZones?: { front?: number; rear?: number; sides?: number }
  }
  body?: { material?: string; crumpleFactor?: number }
}

export function useVehicleLoader() {
  function validate(data: unknown): data is VehicleFile {
    if (!data || typeof data !== 'object') return false
    const d = data as Record<string, unknown>
    if (!Number.isInteger(d.version) || (d.version as number) < 1) return false
    if (typeof d.name !== 'string' || !d.name) return false
    if (!Array.isArray(d.nodes) || d.nodes.length < 4) return false
    if (!Array.isArray(d.beams) || d.beams.length < 1) return false

    const nodeIds = new Set<number>()
    for (const node of d.nodes) {
      if (!node || typeof node !== 'object') return false
      const n = node as Record<string, unknown>
      if (!Number.isInteger(n.id) || (n.id as number) < 0 || nodeIds.has(n.id as number)) return false
      if (![n.x, n.y, n.z, n.mass].every(value => typeof value === 'number' && Number.isFinite(value))) return false
      if ((n.mass as number) <= 0) return false
      if (n.fixed !== undefined && typeof n.fixed !== 'boolean') return false
      if (n.collision !== undefined && typeof n.collision !== 'boolean') return false
      nodeIds.add(n.id as number)
    }

    const beamIds = new Set<number>()
    for (const beam of d.beams) {
      if (!beam || typeof beam !== 'object') return false
      const b = beam as Record<string, unknown>
      if (!Number.isInteger(b.id) || (b.id as number) < 0 || beamIds.has(b.id as number)) return false
      if (!Number.isInteger(b.nodeA) || !Number.isInteger(b.nodeB)) return false
      if (!nodeIds.has(b.nodeA as number) || !nodeIds.has(b.nodeB as number) || b.nodeA === b.nodeB) return false
      for (const key of ['stiffness', 'damping', 'strength']) {
        if (b[key] !== undefined && (typeof b[key] !== 'number' || !Number.isFinite(b[key] as number) || (b[key] as number) < 0)) return false
      }
      beamIds.add(b.id as number)
    }
    return true
  }

  function validateComponentCompatibility(def: VehicleFile): string[] {
    const errors: string[] = []

    // Check gear ratios are descending (each gear should be lower than the previous)
    if (def.transmission?.gearRatios && def.transmission.gearRatios.length > 1) {
      for (let i = 1; i < def.transmission.gearRatios.length; i++) {
        const current = def.transmission.gearRatios[i]!
        const previous = def.transmission.gearRatios[i - 1]!
        if (current >= previous) {
          errors.push(`Gear ratio ${i + 1} (${current}) should be lower than gear ${i} (${previous})`)
        }
      }
    }

    // Check engine RPM ranges make sense
    if (def.engine) {
      if (def.engine.idleRpm != null && def.engine.redlineRpm != null) {
        if (def.engine.idleRpm >= def.engine.redlineRpm) {
          errors.push(`idleRpm (${def.engine.idleRpm}) must be less than redlineRpm (${def.engine.redlineRpm})`)
        }
      }
      if (def.engine.revLimiterRpm != null && def.engine.redlineRpm != null) {
        if (def.engine.revLimiterRpm < def.engine.redlineRpm) {
          errors.push(`revLimiterRpm (${def.engine.revLimiterRpm}) should be >= redlineRpm (${def.engine.redlineRpm})`)
        }
      }
    }

    // Check suspension wheel count matches node layout (rough check: wheels array should have 4 for a car)
    const wheelCount = def.suspension?.wheels?.length
    if (wheelCount != null && wheelCount < 1) {
      errors.push('suspension.wheels must have at least 1 wheel')
    }

    // Check tire count matches wheel count
    const tireCount = def.tires?.length
    if (wheelCount != null && tireCount != null && tireCount !== wheelCount) {
      errors.push(`tire count (${tireCount}) does not match wheel count (${wheelCount})`)
    }

    // Check brake count matches wheel count
    const brakeCount = def.brakes?.length
    if (wheelCount != null && brakeCount != null && brakeCount !== wheelCount) {
      errors.push(`brake count (${brakeCount}) does not match wheel count (${wheelCount})`)
    }

    return errors
  }

  function validateDrivetrainConfig(def: VehicleFile): { valid: boolean; errors: string[] } {
    const errors: string[] = []

    if (!def.transmission) {
      return { valid: true, errors: [] }
    }

    const t = def.transmission

    // Automatic requires shift RPMs
    if (t.mode === 'automatic') {
      if (t.autoShiftUpRpm == null) {
        errors.push('automatic transmission requires autoShiftUpRpm')
      }
      if (t.autoShiftDownRpm == null) {
        errors.push('automatic transmission requires autoShiftDownRpm')
      }
      if (t.autoShiftUpRpm != null && t.autoShiftDownRpm != null && t.autoShiftDownRpm >= t.autoShiftUpRpm) {
        errors.push('autoShiftDownRpm must be less than autoShiftUpRpm')
      }
    }

    // Gear ratios required
    if (!t.gearRatios || t.gearRatios.length === 0) {
      errors.push('transmission.gearRatios required')
    }

    // Differential type valid
    if (t.differential?.type && !['open', 'locked', 'lsd'].includes(t.differential.type)) {
      errors.push(`invalid differential type: ${t.differential.type}`)
    }

    return { valid: errors.length === 0, errors }
  }

  function validateThermalConfig(def: VehicleFile): { valid: boolean; errors: string[] } {
    const errors: string[] = []

    if (!def.engine) {
      return { valid: true, errors: [] }
    }

    const cs = def.engine.coolingSystem
    if (cs) {
      if (cs.fanTemp != null && cs.fanTemp < 50) {
        errors.push('coolingSystem.fanTemp must be >= 50')
      }
      if (cs.coolantCapacity != null && cs.coolantCapacity < 0.1) {
        errors.push('coolingSystem.coolantCapacity must be > 0.1')
      }
    }

    const os = def.engine.oilSystem
    if (os) {
      if (os.oilCapacity != null && os.oilCapacity < 0.5) {
        errors.push('oilSystem.oilCapacity must be >= 0.5')
      }
      if (os.pumpPressure != null && os.pumpPressure < 10) {
        errors.push('oilSystem.pumpPressure must be >= 10')
      }
    }

    return { valid: errors.length === 0, errors }
  }

  function toPhysicsDefinition(data: VehicleFile): VehicleDefinition {
    const nodes: VehicleNodeDef[] = data.nodes.map(n => ({
      id: n.id,
      x: n.x,
      y: n.y,
      z: n.z,
      mass: n.mass,
      fixed: n.fixed ?? false,
    }))

    const beams: VehicleBeamDef[] = data.beams.map(b => ({
      id: b.id,
      nodeA: b.nodeA,
      nodeB: b.nodeB,
      stiffness: b.stiffness ?? 1000,
      damping: b.damping ?? 0.5,
      strength: b.strength ?? 1000,
    }))

    return { nodes, beams }
  }

  async function loadFromUrl(url: string): Promise<VehicleFile> {
    const res = await fetch(url)
    if (!res.ok) throw new Error(`Failed to load vehicle: ${res.status}`)
    const data = await res.json()
    if (!validate(data)) throw new Error('Invalid vehicle definition')
    const compatibilityErrors = [
      ...validateComponentCompatibility(data),
      ...validateDrivetrainConfig(data).errors,
      ...validateThermalConfig(data).errors,
    ]
    if (compatibilityErrors.length > 0) {
      throw new Error(`Invalid vehicle configuration: ${compatibilityErrors.join('; ')}`)
    }
    logDebug('vehicle-loader:loaded', { name: data.name, version: data.version })
    return data
  }

  async function loadMultipleFromUrls(urls: string[]): Promise<VehicleFile[]> {
    const results = await Promise.allSettled(
      urls.map(url => loadFromUrl(url))
    )
    const vehicles: VehicleFile[] = []
    for (const r of results) {
      if (r.status === 'fulfilled') {
        vehicles.push(r.value)
      }
    }
    return vehicles
  }

  function loadFromLocalStorage(): VehicleFile | null {
    if (typeof localStorage === 'undefined') return null
    try {
      const raw = localStorage.getItem(STORAGE_KEY)
      if (!raw) return null
      const data = JSON.parse(raw)
      return validate(data) ? data : null
    } catch {
      return null
    }
  }

  function saveToLocalStorage(data: VehicleFile) {
    if (typeof localStorage === 'undefined' || !validate(data)) return false
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(data))
      return true
    } catch { return false }
  }

  return {
    validate,
    validateComponentCompatibility,
    validateDrivetrainConfig,
    validateThermalConfig,
    toPhysicsDefinition,
    loadFromUrl,
    loadMultipleFromUrls,
    loadFromLocalStorage,
    saveToLocalStorage,
  }
}
