import { defineStore } from 'pinia'

export type PlayModeId = 'freeroam' | 'drag-race'

export interface PlayModeConfig {
  id: PlayModeId
  label: string
  showStaging: boolean
  requiresVehicle: boolean
}

export const PLAY_MODES: Record<PlayModeId, PlayModeConfig> = {
  freeroam: {
    id: 'freeroam',
    label: 'Freeroam',
    showStaging: false,
    requiresVehicle: true,
  },
  'drag-race': {
    id: 'drag-race',
    label: 'Drag Race',
    showStaging: true,
    requiresVehicle: true,
  },
}

export const usePlayModeStore = defineStore('playMode', {
  state: () => ({
    mode: 'freeroam' as PlayModeId,
    isRunning: false,
  }),

  getters: {
    currentMode: (state): PlayModeConfig => PLAY_MODES[state.mode],
    canExit: () => true,
  },

  actions: {
    setMode(mode: PlayModeId) {
      this.mode = mode
      this.isRunning = true
    },
    reset() {
      this.mode = 'freeroam'
      this.isRunning = false
    },
    $reset() {
      this.reset()
    },
  },
})