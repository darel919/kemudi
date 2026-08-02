import { defineStore } from 'pinia'

export type CameraModeId =
  | 'interior'
  | 'hood'
  | 'grill'
  | 'exterior'
  | 'far-exterior'
  | 'cinematic'
  | 'free'

export interface CameraModeConfig {
  id: CameraModeId
  label: string
  requiresVehicle: boolean
  isChase: boolean
}

export const CAMERA_MODES: Record<CameraModeId, CameraModeConfig> = {
  interior: { id: 'interior', label: 'Interior', requiresVehicle: true, isChase: false },
  hood: { id: 'hood', label: 'Hood', requiresVehicle: true, isChase: false },
  grill: { id: 'grill', label: 'Grill', requiresVehicle: true, isChase: false },
  exterior: { id: 'exterior', label: 'Exterior', requiresVehicle: true, isChase: true },
  'far-exterior': { id: 'far-exterior', label: 'Far Exterior', requiresVehicle: true, isChase: true },
  cinematic: { id: 'cinematic', label: 'Cinematic', requiresVehicle: true, isChase: false },
  free: { id: 'free', label: 'Free', requiresVehicle: false, isChase: false },
}

export const useCameraStore = defineStore('camera', {
  state: () => ({
    mode: 'exterior' as CameraModeId,
    distance: 6,
    height: 2,
    yaw: 0,
    pitch: 0,
    fov: 60,
    isInteriorAdjusted: false,
  }),

  getters: {
    currentMode: (state): CameraModeConfig => CAMERA_MODES[state.mode],
    canEnterFree: () => true,
  },

  actions: {
    setMode(mode: CameraModeId) {
      this.mode = mode
    },

    setDistance(d: number) {
      this.distance = Math.max(1, Math.min(50, d))
    },

    setHeight(h: number) {
      this.height = Math.max(0, Math.min(10, h))
    },

    setYaw(y: number) {
      this.yaw = y
    },

    setPitch(p: number) {
      this.pitch = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, p))
    },

    setFov(f: number) {
      this.fov = Math.max(30, Math.min(120, f))
    },

    $reset() {
      this.mode = 'exterior'
      this.distance = 6
      this.height = 2
      this.yaw = 0
      this.pitch = 0
      this.fov = 60
      this.isInteriorAdjusted = false
    },
  },
})