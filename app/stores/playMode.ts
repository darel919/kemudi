import { defineStore } from 'pinia'
import type { PlayModeId, PlayModeLifecycle, DragRacePhase, DragRaceResult } from '~/types/play-mode'

// Re-export original PlayModeId so existing imports keep working
export type { PlayModeId } from '~/types/play-mode'

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
    lifecycle: 'idle' as PlayModeLifecycle,
    dragRacePhase: 'none' as DragRacePhase,
    dragRaceResult: null as DragRaceResult | null,
  }),

  getters: {
    currentMode: (state): PlayModeConfig => PLAY_MODES[state.mode],
    canExit: (state): boolean => state.lifecycle !== 'loading',
  },

  actions: {
    setMode(mode: PlayModeId) {
      this.mode = mode
      this.isRunning = true
      this.lifecycle = 'active'
    },

    setLifecycle(lifecycle: PlayModeLifecycle) {
      this.lifecycle = lifecycle
      this.isRunning = lifecycle === 'active'
    },

    setDragRacePhase(phase: DragRacePhase) {
      this.dragRacePhase = phase
    },

    setDragRaceResult(result: DragRaceResult | null) {
      this.dragRaceResult = result
    },

    reset() {
      this.mode = 'freeroam'
      this.isRunning = false
      this.lifecycle = 'idle'
      this.dragRacePhase = 'none'
      this.dragRaceResult = null
    },

    $reset() {
      this.reset()
    },
  },
})
