import type {
  VehicleDefinition,
  VehicleNodeDef,
  VehicleBeamDef,
  VehicleBodyDefinition,
  VehicleEngineDefinition,
  VehicleFuelDefinition,
  VehicleSafetyDefinition,
  VehicleSuspensionDefinition,
  VehicleTireDefinition,
  VehicleTransmissionDefinition,
} from '~/types/physics'
import { logDebug } from '~/utils/debug'

const STORAGE_KEY = 'kemudi:vehicles'
const VEHICLE_CACHE_LIMIT = 16
const vehicleCache = new Map<string, VehicleFile>()

interface WheelConfig {
  springRate?: number
  damping?: number
  reboundDamping?: number
  restLength?: number
  travel?: number
  unsprungMass?: number
  tireRadius?: number
}

export interface VehicleFile {
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
    bumpStopRate?: number
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
  body?: { material?: string; crumpleFactor?: number; bodyMesh?: string }
  safety_systems?: {
    abs?: { enabled?: boolean }
    traction_control?: { enabled?: boolean }
    vsc_esc?: { enabled?: boolean }
    adas?: { forward_collision_warning?: boolean; automatic_emergency_braking?: boolean }
  }
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
    if (d.body !== undefined) {
      if (!d.body || typeof d.body !== 'object') return false
      const body = d.body as Record<string, unknown>
      if (body.material !== undefined && typeof body.material !== 'string') return false
      if (body.crumpleFactor !== undefined && (typeof body.crumpleFactor !== 'number' || !Number.isFinite(body.crumpleFactor) || body.crumpleFactor < 0 || body.crumpleFactor > 1)) return false
      if (body.bodyMesh !== undefined && typeof body.bodyMesh !== 'string') return false
    }
    if (d.safety_systems !== undefined) {
      if (!d.safety_systems || typeof d.safety_systems !== 'object') return false
      const safety = d.safety_systems as Record<string, unknown>
      for (const key of ['abs', 'traction_control', 'vsc_esc', 'adas']) {
        const system = safety[key]
        if (system !== undefined && (!system || typeof system !== 'object')) return false
        if (system && typeof system === 'object') {
          for (const [field, value] of Object.entries(system as Record<string, unknown>)) {
            if (field === 'enabled' || field === 'forward_collision_warning' || field === 'automatic_emergency_braking') {
              if (typeof value !== 'boolean') return false
            }
          }
        }
      }
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

    if (t.mode !== undefined && t.mode !== 'manual' && t.mode !== 'automatic') {
      errors.push(`invalid transmission mode: ${t.mode}`)
    }
    if (t.gearRatios?.some(ratio => !Number.isFinite(ratio) || ratio <= 0)) {
      errors.push('transmission.gearRatios must contain finite positive values')
    }
    for (const [name, value] of [
      ['finalDrive', t.finalDrive],
      ['reverseRatio', t.reverseRatio],
      ['shiftDelay', t.shiftDelay],
      ['autoShiftUpRpm', t.autoShiftUpRpm],
      ['autoShiftDownRpm', t.autoShiftDownRpm],
    ] as const) {
      if (value !== undefined && (!Number.isFinite(value) || (name === 'shiftDelay' && value < 0))) {
        errors.push(`transmission.${name} must be finite${name === 'shiftDelay' ? ' and non-negative' : ''}`)
      }
    }

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
    if (t.differential?.type && !['open', 'locked', 'lsd', 'limited_slip'].includes(t.differential.type)) {
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
      collision: n.collision,
    }))

    const crumpleFactor = Math.max(0, Math.min(1, data.body?.crumpleFactor ?? 0.5))
    const beams: VehicleBeamDef[] = data.beams.map(b => ({
      id: b.id,
      nodeA: b.nodeA,
      nodeB: b.nodeB,
      stiffness: b.stiffness ?? 1000,
      damping: b.damping ?? 0.5,
      // A higher body crumple factor lowers the beam yield threshold while
      // preserving the authored beam strength as the baseline.
      strength: (b.strength ?? 1000) * (1 - crumpleFactor * 0.45),
    }))

    const runtimeEngine: VehicleEngineDefinition = {
      idleRpm: data.engine?.idleRpm ?? 800,
      redlineRpm: data.engine?.redlineRpm ?? 7000,
      revLimiterRpm: data.engine?.revLimiterRpm ?? 7200,
      throttleResponse: data.engine?.throttleResponse ?? 0.8,
      engineBraking: Math.max(0.1, data.engine?.engineBraking ?? 0.3) * 100,
      torqueCurve: data.engine?.torqueCurve ?? [[0, 100], [1000, 150], [3000, 250], [7000, 160]],
    }
    const diff = data.transmission?.differential?.type
    const runtimeTransmission: VehicleTransmissionDefinition = {
      mode: data.transmission?.mode === 'automatic' ? 'automatic' : 'manual',
      gearRatios: data.transmission?.gearRatios ?? [3.5, 2.1, 1.4, 1, 0.7],
      finalDrive: data.transmission?.finalDrive ?? 3.7,
      reverseRatio: data.transmission?.reverseRatio ?? -3.2,
      shiftDelay: data.transmission?.shiftDelay ?? 0.15,
      autoShiftUpRpm: data.transmission?.autoShiftUpRpm,
      autoShiftDownRpm: data.transmission?.autoShiftDownRpm,
      differential: {
        type: diff === 'locked' ? 'locked' : diff === 'limited_slip' || diff === 'lsd' ? 'limited_slip' : 'open',
        bias: data.transmission?.differential?.bias ?? 0.5,
      },
    }
    const wheels = data.suspension?.wheels ?? []
    const runtimeSuspension: VehicleSuspensionDefinition = {
      antiRollBarStiffness: data.suspension?.antiRollBarStiffness,
      bumpStopRate: data.suspension?.bumpStopRate,
      wheels: Array.from({ length: 4 }, (_, index) => {
        const wheel = wheels[index] ?? wheels[0]
        return {
          springRate: wheel?.springRate ?? data.suspension?.springRate ?? 30000,
          damping: wheel?.damping ?? data.suspension?.damping ?? 4000,
          reboundDamping: wheel?.reboundDamping ?? data.suspension?.damping ?? 2500,
          restLength: wheel?.restLength ?? 0.35,
          travel: wheel?.travel ?? data.suspension?.travel ?? 0.2,
          tireRadius: wheel?.tireRadius ?? 0.33,
          unsprungMass: wheel?.unsprungMass ?? 15,
        }
      }),
    }
    const runtimeTires: VehicleTireDefinition[] = Array.from({ length: 4 }, (_, index) => {
      const tire = data.tires?.[index] ?? data.tires?.[0]
      const compound = tire?.compound?.toLowerCase()
      return {
        compound: compound === 'performance' ? 'sport' : compound === 'offroad' || compound === 'mud' || compound === 'snow' ? compound : compound === 'sport' ? 'sport' : 'street',
        nominalPressure: tire?.nominalPressure ?? 32,
      }
    })
    const runtimeFuel: VehicleFuelDefinition = {
      capacity: data.fuel?.capacity ?? 60,
      consumptionRate: data.fuel?.consumptionRate ?? 0.01,
      idleConsumptionRate: data.fuel?.idleConsumptionRate ?? 0.0005,
    }
    const runtimeSafety: VehicleSafetyDefinition = {
      abs: data.safety_systems?.abs?.enabled ?? false,
      tractionControl: data.safety_systems?.traction_control?.enabled ?? false,
      vsc: data.safety_systems?.vsc_esc?.enabled ?? false,
      adasForwardCollisionWarning: data.safety_systems?.adas?.forward_collision_warning ?? false,
      adasAutomaticEmergencyBraking: data.safety_systems?.adas?.automatic_emergency_braking ?? false,
    }
    const runtimeBody: VehicleBodyDefinition = {
      material: data.body?.material ?? 'steel',
      crumpleFactor,
      bodyMesh: data.body?.bodyMesh,
    }
    const triangles: [number, number, number][] = data.nodes.length >= 8
      ? [[data.nodes[0]!.id, data.nodes[1]!.id, data.nodes[3]!.id], [data.nodes[0]!.id, data.nodes[3]!.id, data.nodes[2]!.id], [data.nodes[4]!.id, data.nodes[6]!.id, data.nodes[7]!.id], [data.nodes[4]!.id, data.nodes[7]!.id, data.nodes[5]!.id]]
      : []
    // The 12-node car/truck layout adds a roof layer. Without area
    // constraints on that layer, the upper nodes can preserve beam lengths
    // by spreading sideways until the visual body becomes a flat slab.
    if (data.nodes.length >= 12) {
      triangles.push(
        [data.nodes[4]!.id, data.nodes[5]!.id, data.nodes[9]!.id],
        [data.nodes[4]!.id, data.nodes[9]!.id, data.nodes[8]!.id],
        [data.nodes[6]!.id, data.nodes[10]!.id, data.nodes[11]!.id],
        [data.nodes[6]!.id, data.nodes[11]!.id, data.nodes[7]!.id],
        [data.nodes[8]!.id, data.nodes[9]!.id, data.nodes[11]!.id],
        [data.nodes[8]!.id, data.nodes[11]!.id, data.nodes[10]!.id],
      )
    }

    return {
      nodes,
      beams,
      triangles,
      engine: runtimeEngine,
      transmission: runtimeTransmission,
      suspension: runtimeSuspension,
      tires: runtimeTires,
      fuel: runtimeFuel,
      safety: runtimeSafety,
      body: runtimeBody,
    }
  }

  async function loadFromUrl(url: string): Promise<VehicleFile> {
    const cached = vehicleCache.get(url)
    if (cached) return cached
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
    if (vehicleCache.size >= VEHICLE_CACHE_LIMIT) {
      const oldest = vehicleCache.keys().next().value
      if (oldest) vehicleCache.delete(oldest)
    }
    vehicleCache.set(url, data)
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
