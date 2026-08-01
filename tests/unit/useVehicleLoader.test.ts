import { describe, it, expect, vi } from 'vitest'

vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

import { useVehicleLoader } from '../../app/composables/useVehicleLoader'
import basicCar from '../../public/vehicles/basic_car.vehicle.json'
import basicTruck from '../../public/vehicles/basic_truck.vehicle.json'
import basicAtv from '../../public/vehicles/basic_atv.vehicle.json'
import premiumSportscar from '../../public/vehicles/premium_sportscar.vehicle.json'

const validVehicle = {
  version: 1,
  name: 'Test Car',
  nodes: [
    { id: 0, x: -1, y: 0, z: -1, mass: 50 },
    { id: 1, x: 1, y: 0, z: -1, mass: 50 },
    { id: 2, x: -1, y: 0, z: 1, mass: 50 },
    { id: 3, x: 1, y: 0, z: 1, mass: 50 },
  ],
  beams: [
    { id: 0, nodeA: 0, nodeB: 1, stiffness: 10000, damping: 0.5, strength: 2000 },
    { id: 1, nodeA: 2, nodeB: 3, stiffness: 10000, damping: 0.5, strength: 2000 },
  ],
}

describe('useVehicleLoader', () => {
  const loader = useVehicleLoader()

  it('validates correct vehicle data', () => {
    expect(loader.validate(validVehicle)).toBe(true)
  })

  it('rejects missing version', () => {
    const data = { ...validVehicle, version: undefined }
    expect(loader.validate(data)).toBe(false)
  })

  it('rejects too few nodes', () => {
    const data = { ...validVehicle, nodes: [{ id: 0, x: 0, y: 0, z: 0, mass: 1 }] }
    expect(loader.validate(data)).toBe(false)
  })

  it('rejects no beams', () => {
    const data = { ...validVehicle, beams: [] }
    expect(loader.validate(data)).toBe(false)
  })

  it('rejects duplicate node ids and dangling beam references', () => {
    const duplicate = {
      ...validVehicle,
      nodes: validVehicle.nodes.map((node, index) => ({ ...node, id: index === 1 ? 0 : node.id })),
    }
    expect(loader.validate(duplicate)).toBe(false)

    const dangling = {
      ...validVehicle,
      beams: [{ ...validVehicle.beams[0], nodeB: 99 }],
    }
    expect(loader.validate(dangling)).toBe(false)
  })

  it('rejects out-of-range body crumple factors and malformed safety flags', () => {
    expect(loader.validate({ ...validVehicle, body: { crumpleFactor: 1.2 } })).toBe(false)
    expect(loader.validate({ ...validVehicle, safety_systems: { adas: { forward_collision_warning: 'yes' } } })).toBe(false)
  })

  it('converts to physics definition', () => {
    const def = loader.toPhysicsDefinition(validVehicle)
    expect(def.nodes).toHaveLength(4)
    expect(def.beams).toHaveLength(2)
    expect(def.nodes[0].x).toBe(-1)
    expect(def.beams[0].stiffness).toBe(10000)
    expect(def.engine?.torqueCurve.length).toBeGreaterThan(0)
    expect(def.suspension?.wheels).toHaveLength(4)
    expect(def.body?.material).toBe('steel')
    expect(def.body?.crumpleFactor).toBeGreaterThanOrEqual(0)
  })

  it('applies defaults for missing optional fields', () => {
    const minimal = {
      version: 1,
      name: 'Minimal',
      nodes: [
        { id: 0, x: 0, y: 0, z: 0, mass: 1 },
        { id: 1, x: 1, y: 0, z: 0, mass: 1 },
        { id: 2, x: 0, y: 1, z: 0, mass: 1 },
        { id: 3, x: 1, y: 1, z: 0, mass: 1 },
      ],
      beams: [{ id: 0, nodeA: 0, nodeB: 1 }],
    }
    const def = loader.toPhysicsDefinition(minimal)
    expect(def.beams[0].stiffness).toBe(1000) // default
      expect(def.beams[0].damping).toBe(0.5) // default
      expect(def.transmission?.mode).toBe('manual')
  })

  describe('sample vehicles', () => {
    it('validates basic_car', () => {
      expect(loader.validate(basicCar)).toBe(true)
    })

    it('validates basic_truck', () => {
      expect(loader.validate(basicTruck)).toBe(true)
    })

    it('validates basic_atv', () => {
      expect(loader.validate(basicAtv)).toBe(true)
    })

    it('converts basic_car to physics definition', () => {
      const def = loader.toPhysicsDefinition(basicCar as any)
      expect(def.nodes).toHaveLength(8)
      expect(def.beams).toHaveLength(16)
    })

    it('converts basic_truck to physics definition', () => {
      const def = loader.toPhysicsDefinition(basicTruck as any)
      expect(def.nodes).toHaveLength(12)
      expect(def.beams).toHaveLength(22)
    })

    it('converts basic_atv to physics definition', () => {
      const def = loader.toPhysicsDefinition(basicAtv as any)
      expect(def.nodes).toHaveLength(10)
      expect(def.beams).toHaveLength(17)
    })
  })

  describe('component compatibility', () => {
    it('catches bad gear ratios (non-descending)', () => {
      const badGear = {
        ...validVehicle,
        transmission: { gearRatios: [3.5, 2.1, 2.5, 1.0] },
      }
      const errors = loader.validateComponentCompatibility(badGear)
      expect(errors.length).toBeGreaterThan(0)
      expect(errors[0]).toContain('Gear ratio 3')
    })

    it('catches mismatched wheel and tire count', () => {
      const mismatched = {
        ...validVehicle,
        suspension: {
          wheels: [
            { springRate: 30000, damping: 4000, restLength: 0.3, travel: 0.2, unsprungMass: 15, tireRadius: 0.33 },
            { springRate: 30000, damping: 4000, restLength: 0.3, travel: 0.2, unsprungMass: 15, tireRadius: 0.33 },
          ],
        },
        tires: [{ compound: 'street' }],
      }
      const errors = loader.validateComponentCompatibility(mismatched)
      expect(errors.some(e => e.includes('tire count'))).toBe(true)
    })

    it('catches idleRpm >= redlineRpm', () => {
      const badEngine = {
        ...validVehicle,
        engine: { idleRpm: 8000, redlineRpm: 7000 },
      }
      const errors = loader.validateComponentCompatibility(badEngine)
      expect(errors.some(e => e.includes('idleRpm'))).toBe(true)
    })

    it('passes with valid component config', () => {
      const good = {
        ...basicCar,
        suspension: { wheels: [{}, {}, {}, {}] },
        tires: [{}, {}, {}, {}],
        brakes: [{}, {}, {}, {}],
      }
      const errors = loader.validateComponentCompatibility(good)
      expect(errors).toHaveLength(0)
    })
  })

  describe('drivetrain validation', () => {
    it('requires autoShiftUpRpm for automatic', () => {
      const result = loader.validateDrivetrainConfig({
        ...validVehicle,
        transmission: { mode: 'automatic', gearRatios: [3.5, 2.1] },
      })
      expect(result.valid).toBe(false)
      expect(result.errors.some(e => e.includes('autoShiftUpRpm'))).toBe(true)
    })

    it('validates autoShiftDownRpm < autoShiftUpRpm', () => {
      const result = loader.validateDrivetrainConfig({
        ...validVehicle,
        transmission: {
          mode: 'automatic', gearRatios: [3.5, 2.1],
          autoShiftUpRpm: 2000, autoShiftDownRpm: 3000,
        },
      })
      expect(result.valid).toBe(false)
      expect(result.errors.some(e => e.includes('less than'))).toBe(true)
    })

    it('passes for valid manual config', () => {
      const result = loader.validateDrivetrainConfig({
        ...validVehicle,
        transmission: { mode: 'manual', gearRatios: [3.5, 2.1] },
      })
      expect(result.valid).toBe(true)
    })

    it('accepts the sample sportscar limited-slip differential', () => {
      const result = loader.validateDrivetrainConfig(premiumSportscar as any)
      expect(result.valid).toBe(true)
      expect(result.errors).toHaveLength(0)
    })
  })

  describe('safety_systems validation', () => {
    it('accepts vehicle without safety_systems (optional)', () => {
      expect(loader.validate(validVehicle)).toBe(true)
      expect((validVehicle as any).safety_systems).toBeUndefined()
    })

    it('validates basic_car has no safety_systems (economy car)', () => {
      expect(loader.validate(basicCar)).toBe(true)
      const ss = (basicCar as any).safety_systems
      expect(ss).toBeUndefined()
    })

    it('validates basic_truck has ABS only', () => {
      expect(loader.validate(basicTruck)).toBe(true)
      const ss = (basicTruck as any).safety_systems
      expect(ss).toBeDefined()
      expect(ss.abs.enabled).toBe(true)
      expect(ss.abs.channels).toBe(4)
      expect(ss.traction_control).toBeUndefined()
      expect(ss.vsc_esc).toBeUndefined()
      expect(ss.adas).toBeUndefined()
    })

    it('validates basic_atv safety_systems', () => {
      expect(loader.validate(basicAtv)).toBe(true)
      const ss = (basicAtv as any).safety_systems
      expect(ss).toBeDefined()
      expect(ss.abs.channels).toBe(2)
      expect(ss.traction_control).toBeUndefined()
      expect(ss.vsc_esc).toBeUndefined()
      expect(ss.adas).toBeUndefined()
    })

    it('validates premium_sportscar has full safety suite', () => {
      expect(loader.validate(premiumSportscar)).toBe(true)
      const ss = (premiumSportscar as any).safety_systems
      expect(ss).toBeDefined()
      expect(ss.abs.enabled).toBe(true)
      expect(ss.abs.channels).toBe(4)
      expect(ss.traction_control.enabled).toBe(true)
      expect(ss.traction_control.brake_intervention).toBe(true)
      expect(ss.vsc_esc.enabled).toBe(true)
      expect(ss.vsc_esc.individual_brake_vectoring).toBe(true)
      expect(ss.adas.forward_collision_warning).toBe(true)
      expect(ss.adas.automatic_emergency_braking).toBe(true)
      expect(ss.adas.adaptive_cruise_control).toBe(true)
      expect(ss.adas.blind_spot_warning).toBe(true)
      expect(ss.adas.lane_departure_assist).toBe(true)
      expect(ss.adas.parking_sensors).toBe(true)
    })

    it('accepts safety_systems with invalid channels (schema enforced elsewhere)', () => {
      // The runtime validate() only checks core fields; enum constraints are enforced by JSON Schema loader
      const bad = {
        ...validVehicle,
        safety_systems: { abs: { enabled: true, channels: 3 } },
      }
      expect(loader.validate(bad)).toBe(true)
    })
  })

  describe('thermal config validation', () => {
    it('rejects low fanTemp', () => {
      const result = loader.validateThermalConfig({
        ...validVehicle,
        engine: { coolingSystem: { fanTemp: 20 } },
      })
      expect(result.valid).toBe(false)
      expect(result.errors.some(e => e.includes('fanTemp'))).toBe(true)
    })

    it('rejects low oilCapacity', () => {
      const result = loader.validateThermalConfig({
        ...validVehicle,
        engine: { oilSystem: { oilCapacity: 0.1 } },
      })
      expect(result.valid).toBe(false)
      expect(result.errors.some(e => e.includes('oilCapacity'))).toBe(true)
    })

    it('passes with no engine config', () => {
      const result = loader.validateThermalConfig(validVehicle)
      expect(result.valid).toBe(true)
    })

    it('passes with valid thermal config', () => {
      const result = loader.validateThermalConfig({
        ...validVehicle,
        engine: {
          coolingSystem: { fanTemp: 95, coolantCapacity: 1.5 },
          oilSystem: { oilCapacity: 4.0, pumpPressure: 45 },
        },
      })
      expect(result.valid).toBe(true)
    })
  })
})
