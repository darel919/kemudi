import { defineStore } from 'pinia'
import type { MapDefinition } from '~/types/map-schema'
import { KNOWN_MAP_IDS, type BaseMapId } from '~/types/base-map'

export type TransmissionMode = 'manual' | 'automatic'
export type IgnitionState = 'off' | 'accessory' | 'running'

export interface VehicleConfig {
  id: string
  name: string
  assetPath: string
  transmission: TransmissionMode
  transmissionOptions?: TransmissionMode[]
}

/** Minimal fallback map — only used before async map loading completes. */
function emptyMap(): MapDefinition {
  const defaultId = KNOWN_MAP_IDS[0]
  return {
    id: defaultId,
    label: defaultId,
    description: '',
    version: 1,
    size: { width: 800, depth: 800 },
    segments: 128,
    preview: { bgColor: 0x3b4650, pattern: 'grid', patternColor: 0x5a6a7a, patternOpacity: 0.18 },
    terrain: {
      heightmap: null, heightScale: 0, color: 0x3b4650, roughness: 0.76,
      groundFriction: 0.94, procedural: null, layers: [],
    },
    roads: [], objects: [], spawnPoints: [],
  }
}

export const useVehicleSessionStore = defineStore('vehicleSession', {
  state: () => ({
    activeVehicleId: null as string | null,
    vehicle: null as VehicleConfig | null,
    mapId: KNOWN_MAP_IDS[0] as string,
    map: emptyMap() as MapDefinition,
    fuelLevel: 1.0,
    damage: 0,
    isRunning: false,
    ignition: 'off' as IgnitionState,
  }),

  getters: {
    hasVehicle: (state) => state.vehicle !== null,
    isDriveable: (state) => {
      if (!state.vehicle) return false
      if (state.fuelLevel <= 0) return false
      if (state.damage >= 1) return false
      return true
    },
  },

  actions: {
    spawnVehicle(config: VehicleConfig, mapDef: MapDefinition) {
      this.vehicle = { ...config }
      this.activeVehicleId = config.id
      this.mapId = mapDef.id
      this.map = mapDef
      this.fuelLevel = 1.0
      this.damage = 0
      this.isRunning = false
      this.ignition = 'off'
    },

    removeVehicle() {
      this.vehicle = null
      this.activeVehicleId = null
      this.isRunning = false
      this.ignition = 'off'
    },

    consumeFuel(amount: number) {
      this.fuelLevel = Math.max(0, this.fuelLevel - amount)
    },

    addDamage(amount: number) {
      this.damage = Math.min(1, this.damage + amount)
    },

    advanceIgnition() {
      if (this.ignition === 'off') this.ignition = 'accessory'
      else if (this.ignition === 'accessory') this.ignition = 'running'
      else this.ignition = 'off'
      this.isRunning = this.ignition === 'running'
      return this.ignition
    },

    setMap(mapDef: MapDefinition) {
      this.mapId = mapDef.id
      this.map = mapDef
    },

    setTransmissionMode(mode: TransmissionMode) {
      if (this.vehicle) this.vehicle.transmission = mode
    },

    resetSession() {
      this.removeVehicle()
      this.fuelLevel = 1.0
      this.damage = 0
      this.mapId = KNOWN_MAP_IDS[0]
      this.map = emptyMap()
      this.ignition = 'off'
    },

    $reset() {
      this.resetSession()
    },
  },
})
