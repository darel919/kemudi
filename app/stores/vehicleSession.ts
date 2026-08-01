import { defineStore } from 'pinia'
import type { BaseMapId } from '~/types/base-map'

export type TransmissionMode = 'manual' | 'automatic'
export type IgnitionState = 'off' | 'accessory' | 'running'

export interface VehicleConfig {
  id: string
  name: string
  assetPath: string
  transmission: TransmissionMode
  transmissionOptions?: TransmissionMode[]
}

export const useVehicleSessionStore = defineStore('vehicleSession', {
  state: () => ({
    activeVehicleId: null as string | null,
    vehicle: null as VehicleConfig | null,
    mapId: 'flat' as BaseMapId,
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
    spawnVehicle(config: VehicleConfig, mapId: BaseMapId = 'flat') {
      this.vehicle = { ...config }
      this.activeVehicleId = config.id
      this.mapId = mapId
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

    setMap(mapId: BaseMapId) {
      this.mapId = mapId
    },

    setTransmissionMode(mode: TransmissionMode) {
      if (this.vehicle) this.vehicle.transmission = mode
    },

    resetSession() {
      this.removeVehicle()
      this.fuelLevel = 1.0
      this.damage = 0
      this.mapId = 'flat'
      this.ignition = 'off'
    },

    $reset() {
      this.resetSession()
    },
  },
})
