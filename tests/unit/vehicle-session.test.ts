import { beforeEach, describe, expect, it } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { useVehicleSessionStore } from '../../app/stores/vehicleSession'
import type { MapDefinition } from '../../app/types/map-schema'

const vehicle = {
  id: 'test-car',
  name: 'Test Car',
  assetPath: '/vehicles/basic_car.vehicle.json',
  transmission: 'manual' as const,
}

const testMap: MapDefinition = {
  id: 'dragville',
  label: 'Dragville',
  description: 'Test map',
  version: 1,
  size: { width: 30, depth: 1610 },
  segments: 16,
  preview: { bgColor: 0x2a2a2a, pattern: 'stripe' },
  terrain: {
    heightmap: null, heightScale: 0, color: 0x2a2a2a, roughness: 0.65,
    groundFriction: 0.96, procedural: null, layers: [],
  },
  roads: [], objects: [], spawnPoints: [],
}

describe('vehicle session ignition', () => {
  beforeEach(() => setActivePinia(createPinia()))

  it('enters every level with the engine off', () => {
    const store = useVehicleSessionStore()
    store.spawnVehicle(vehicle, testMap)
    expect(store.ignition).toBe('off')
    expect(store.isRunning).toBe(false)
    expect(store.mapId).toBe('dragville')
  })

  it('advances off → accessory → running → off', () => {
    const store = useVehicleSessionStore()
    store.spawnVehicle(vehicle, testMap)
    expect(store.advanceIgnition()).toBe('accessory')
    expect(store.advanceIgnition()).toBe('running')
    expect(store.isRunning).toBe(true)
    expect(store.advanceIgnition()).toBe('off')
    expect(store.isRunning).toBe(false)
  })
})
