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
  VehicleMassProperties,
  VehicleWeightDistribution,
  VehicleDrivetrainDefinition,
  VehicleAerodynamicsDefinition,
  VehicleBodyGeometry,
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
    yieldStrength?: number; plasticity?: number
  }>
  massProperties?: {
    totalMass?: number
    centerOfMass?: { x?: number; y?: number; z?: number }
    inertia?: { x?: number; y?: number; z?: number }
  }
  weightDistribution?: { front?: number; rear?: number }
  drivetrain?: { layout?: string }
  aerodynamics?: {
    frontalArea?: number; dragCoefficient?: number; liftCoefficient?: number
    centerOfPressure?: { x?: number; y?: number; z?: number }
  }
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
  body?: {
    material?: string; crumpleFactor?: number; bodyMesh?: string
    geometry?: Partial<VehicleBodyGeometry>
  }
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
    if (d.massProperties !== undefined) {
      if (!d.massProperties || typeof d.massProperties !== 'object') return false
      const massProperties = d.massProperties as Record<string, unknown>
      if (massProperties.totalMass !== undefined && !isFinitePositive(massProperties.totalMass)) return false
      if (!validateVector(massProperties.centerOfMass, false)) return false
      if (!validateVector(massProperties.inertia, true)) return false
    }
    if (d.weightDistribution !== undefined) {
      if (!d.weightDistribution || typeof d.weightDistribution !== 'object') return false
      const distribution = d.weightDistribution as Record<string, unknown>
      if (![distribution.front, distribution.rear].every(value => typeof value === 'number' && Number.isFinite(value) && value >= 0 && value <= 1)) return false
      if (Math.abs((distribution.front as number) + (distribution.rear as number) - 1) > 1e-6) return false
    }
    if (d.drivetrain !== undefined) {
      if (!d.drivetrain || typeof d.drivetrain !== 'object') return false
      const drivetrain = d.drivetrain as Record<string, unknown>
      if (drivetrain.layout !== undefined && !['rwd', 'fwd', 'awd'].includes(drivetrain.layout as string)) return false
    }
    if (d.aerodynamics !== undefined) {
      if (!d.aerodynamics || typeof d.aerodynamics !== 'object') return false
      const aero = d.aerodynamics as Record<string, unknown>
      for (const key of ['frontalArea', 'dragCoefficient', 'liftCoefficient']) {
        if (aero[key] !== undefined && (typeof aero[key] !== 'number' || !Number.isFinite(aero[key] as number) || (key !== 'liftCoefficient' && (aero[key] as number) < 0))) return false
      }
      if (!validateVector(aero.centerOfPressure, false)) return false
    }
    if (d.body !== undefined) {
      if (!d.body || typeof d.body !== 'object') return false
      const body = d.body as Record<string, unknown>
      if (body.material !== undefined && typeof body.material !== 'string') return false
      if (body.crumpleFactor !== undefined && (typeof body.crumpleFactor !== 'number' || !Number.isFinite(body.crumpleFactor) || body.crumpleFactor < 0 || body.crumpleFactor > 1)) return false
      if (body.bodyMesh !== undefined && typeof body.bodyMesh !== 'string') return false
      if (body.geometry !== undefined) {
        if (!body.geometry || typeof body.geometry !== 'object') return false
        for (const [key, value] of Object.entries(body.geometry as Record<string, unknown>)) {
          if (!['length', 'width', 'height', 'wheelbase', 'frontTrack', 'rearTrack', 'groundClearance'].includes(key)) return false
          if (typeof value !== 'number' || !Number.isFinite(value) || value <= 0) return false
        }
      }
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

    if (def.drivetrain?.layout !== undefined && !['rwd', 'fwd', 'awd'].includes(def.drivetrain.layout)) {
      errors.push(`invalid drivetrain layout: ${def.drivetrain.layout}`)
    }

    if (!def.transmission) {
      return { valid: errors.length === 0, errors }
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
    const authoredNodes: VehicleNodeDef[] = data.nodes.map(n => ({
      id: n.id,
      x: n.x,
      y: n.y,
      z: n.z,
      mass: n.mass,
      fixed: n.fixed ?? false,
      collision: n.collision,
    }))
    const authoredMass = authoredNodes.reduce((sum, node) => sum + node.mass, 0)
    const targetMass = data.massProperties?.totalMass ?? authoredMass
    const distribution = normalizeWeightDistribution(data.weightDistribution)
    const split = getAxleSplit(authoredNodes)
    const frontNodes = authoredNodes.filter(node => node.z <= split)
    const rearNodes = authoredNodes.filter(node => node.z > split)
    const frontAuthoredMass = frontNodes.reduce((sum, node) => sum + node.mass, 0)
    const rearAuthoredMass = rearNodes.reduce((sum, node) => sum + node.mass, 0)
    const nodes: VehicleNodeDef[] = authoredNodes.map(node => {
      if (!Number.isFinite(targetMass) || targetMass <= 0 || authoredMass <= 0) return node
      if (!data.weightDistribution) return { ...node, mass: node.mass * targetMass / authoredMass }
      const axleMass = node.z <= split ? frontAuthoredMass : rearAuthoredMass
      const targetAxleMass = node.z <= split ? targetMass * distribution.front : targetMass * distribution.rear
      return { ...node, mass: axleMass > 0 ? node.mass * targetAxleMass / axleMass : targetAxleMass / (node.z <= split ? Math.max(frontNodes.length, 1) : Math.max(rearNodes.length, 1)) }
    })
    const runtimeGeometry = resolveBodyGeometry(data.body?.geometry, nodes)
    const runtimeMassProperties: VehicleMassProperties = {
      totalMass: nodes.reduce((sum, node) => sum + node.mass, 0),
      centerOfMass: data.massProperties?.centerOfMass && hasVector(data.massProperties.centerOfMass)
        ? { x: data.massProperties.centerOfMass.x!, y: data.massProperties.centerOfMass.y!, z: data.massProperties.centerOfMass.z! }
        : calculateCenterOfMass(nodes),
      inertia: data.massProperties?.inertia && hasVector(data.massProperties.inertia)
        ? { x: data.massProperties.inertia.x!, y: data.massProperties.inertia.y!, z: data.massProperties.inertia.z! }
        : estimateInertia(nodes),
    }

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
      yieldStrength: b.yieldStrength == null ? undefined : b.yieldStrength * (1 - crumpleFactor * 0.45),
      plasticity: b.plasticity,
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
      geometry: runtimeGeometry,
    }
    const runtimeDrivetrain: VehicleDrivetrainDefinition = {
      layout: data.drivetrain?.layout === 'fwd' || data.drivetrain?.layout === 'awd' ? data.drivetrain.layout : 'rwd',
    }
    const runtimeAero: VehicleAerodynamicsDefinition = {
      frontalArea: data.aerodynamics?.frontalArea ?? Math.max(0.1, runtimeGeometry.width * runtimeGeometry.height * 0.75),
      dragCoefficient: data.aerodynamics?.dragCoefficient ?? 0.32,
      liftCoefficient: data.aerodynamics?.liftCoefficient ?? 0,
      centerOfPressure: data.aerodynamics?.centerOfPressure && hasVector(data.aerodynamics.centerOfPressure)
        ? { x: data.aerodynamics.centerOfPressure.x!, y: data.aerodynamics.centerOfPressure.y!, z: data.aerodynamics.centerOfPressure.z! }
        : { x: runtimeMassProperties.centerOfMass.x, y: runtimeGeometry.groundClearance + runtimeGeometry.height * 0.5, z: runtimeMassProperties.centerOfMass.z },
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
      massProperties: runtimeMassProperties,
      weightDistribution: distribution,
      drivetrain: runtimeDrivetrain,
      aerodynamics: runtimeAero,
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

function isFinitePositive(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value) && value > 0
}

function validateVector(value: unknown, positive: boolean): boolean {
  if (value === undefined) return true
  if (!value || typeof value !== 'object') return false
  const vector = value as Record<string, unknown>
  return ['x', 'y', 'z'].every(key => typeof vector[key] === 'number' && Number.isFinite(vector[key] as number) && (!positive || (vector[key] as number) > 0))
}

function hasVector(value: { x?: number; y?: number; z?: number }): value is { x: number; y: number; z: number } {
  return [value.x, value.y, value.z].every(component => typeof component === 'number' && Number.isFinite(component))
}

function normalizeWeightDistribution(value?: { front?: number; rear?: number }): VehicleWeightDistribution {
  if (value?.front !== undefined && value.rear !== undefined) return { front: value.front, rear: value.rear }
  return { front: 0.5, rear: 0.5 }
}

function getAxleSplit(nodes: VehicleNodeDef[]): number {
  const zValues = nodes.map(node => node.z)
  if (zValues.length === 0) return 0
  return (Math.min(...zValues) + Math.max(...zValues)) * 0.5
}

function calculateCenterOfMass(nodes: VehicleNodeDef[]) {
  const mass = nodes.reduce((sum, node) => sum + node.mass, 0) || 1
  return {
    x: nodes.reduce((sum, node) => sum + node.x * node.mass, 0) / mass,
    y: nodes.reduce((sum, node) => sum + node.y * node.mass, 0) / mass,
    z: nodes.reduce((sum, node) => sum + node.z * node.mass, 0) / mass,
  }
}

function estimateInertia(nodes: VehicleNodeDef[]) {
  const center = calculateCenterOfMass(nodes)
  return {
    x: nodes.reduce((sum, node) => sum + node.mass * ((node.y - center.y) ** 2 + (node.z - center.z) ** 2), 0),
    y: nodes.reduce((sum, node) => sum + node.mass * ((node.x - center.x) ** 2 + (node.z - center.z) ** 2), 0),
    z: nodes.reduce((sum, node) => sum + node.mass * ((node.x - center.x) ** 2 + (node.y - center.y) ** 2), 0),
  }
}

function resolveBodyGeometry(authored: Partial<VehicleBodyGeometry> | undefined, nodes: VehicleNodeDef[]): VehicleBodyGeometry {
  const xValues = nodes.map(node => node.x)
  const yValues = nodes.map(node => node.y)
  const zValues = nodes.map(node => node.z)
  const width = Math.max(0.1, Math.max(...xValues) - Math.min(...xValues))
  const height = Math.max(0.1, Math.max(...yValues) - Math.min(...yValues))
  const length = Math.max(0.1, Math.max(...zValues) - Math.min(...zValues))
  const split = getAxleSplit(nodes)
  const front = nodes.filter(node => node.z <= split)
  const rear = nodes.filter(node => node.z > split)
  const frontZ = front.length ? front.reduce((sum, node) => sum + node.z, 0) / front.length : Math.min(...zValues)
  const rearZ = rear.length ? rear.reduce((sum, node) => sum + node.z, 0) / rear.length : Math.max(...zValues)
  return {
    length: authored?.length ?? length,
    width: authored?.width ?? width,
    height: authored?.height ?? height,
    wheelbase: authored?.wheelbase ?? Math.max(0.1, Math.abs(rearZ - frontZ)),
    frontTrack: authored?.frontTrack ?? width,
    rearTrack: authored?.rearTrack ?? width,
    groundClearance: authored?.groundClearance ?? Math.max(0.05, Math.min(...yValues)),
  }
}
