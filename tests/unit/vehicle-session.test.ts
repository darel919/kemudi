import { beforeEach, describe, expect, it } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { useVehicleSessionStore } from '../../app/stores/vehicleSession'

const vehicle = {
  id: 'test-car',
  name: 'Test Car',
  assetPath: '/vehicles/basic_car.vehicle.json',
  transmission: 'manual' as const,
}

describe('vehicle session ignition', () => {
  beforeEach(() => setActivePinia(createPinia()))

  it('enters every level with the engine off', () => {
    const store = useVehicleSessionStore()
    store.spawnVehicle(vehicle, 'offroad')
    expect(store.ignition).toBe('off')
    expect(store.isRunning).toBe(false)
    expect(store.mapId).toBe('offroad')
  })

  it('advances off → accessory → running → off', () => {
    const store = useVehicleSessionStore()
    store.spawnVehicle(vehicle)
    expect(store.advanceIgnition()).toBe('accessory')
    expect(store.advanceIgnition()).toBe('running')
    expect(store.isRunning).toBe(true)
    expect(store.advanceIgnition()).toBe('off')
    expect(store.isRunning).toBe(false)
  })
})
