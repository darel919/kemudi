import { defineStore } from 'pinia'

export type TransmissionMode = 'manual' | 'automatic'

export interface VehicleConfig {
  id: string
  name: string
  assetPath: string
  transmission: TransmissionMode
}

export const useVehicleSessionStore = defineStore('vehicleSession', {
  state: () => ({
    activeVehicleId: null as string | null,
    vehicle: null as VehicleConfig | null,
    fuelLevel: 1.0,
    damage: 0,
    isRunning: false,
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
    spawnVehicle(config: VehicleConfig) {
      this.vehicle = { ...config }
      this.activeVehicleId = config.id
      this.fuelLevel = 1.0
      this.damage = 0
      this.isRunning = true
    },

    removeVehicle() {
      this.vehicle = null
      this.activeVehicleId = null
      this.isRunning = false
    },

    consumeFuel(amount: number) {
      this.fuelLevel = Math.max(0, this.fuelLevel - amount)
    },

    addDamage(amount: number) {
      this.damage = Math.min(1, this.damage + amount)
    },

    resetSession() {
      this.removeVehicle()
      this.fuelLevel = 1.0
      this.damage = 0
    },

    $reset() {
      this.resetSession()
    },
  },
})