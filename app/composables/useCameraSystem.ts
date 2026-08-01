import * as THREE from 'three'
import { ref, readonly, type Ref } from 'vue'
import { logDebug } from '~/utils/debug'

export type CameraMode =
  | 'exterior'
  | 'hood'
  | 'grill'
  | 'far-exterior'
  | 'interior'
  | 'cinematic'
  | 'free'

const CAMERA_DEFAULTS: Record<CameraMode, Partial<THREE.PerspectiveCamera>> = {
  exterior: { fov: 60, near: 0.1, far: 500 },
  hood: { fov: 55, near: 0.05, far: 300 },
  grill: { fov: 50, near: 0.02, far: 200 },
  'far-exterior': { fov: 50, near: 0.1, far: 800 },
  interior: { fov: 70, near: 0.01, far: 100 },
  cinematic: { fov: 45, near: 0.1, far: 1000 },
  free: { fov: 60, near: 0.1, far: 1000 },
}

export interface CameraSystemHandle {
  camera: THREE.PerspectiveCamera
  currentMode: Readonly<Ref<CameraMode>>
  /** Set camera mode and reset to defaults */
  setMode(mode: CameraMode): void
  /** Update camera position/lookAt for the current frame. Call each frame. */
  update(targetPosition: THREE.Vector3, targetRotation: number, dt: number): void
  dispose(): void
}

const SMOOTHING: Record<CameraMode, number> = {
  exterior: 0.08,
  hood: 0.12,
  grill: 0.15,
  'far-exterior': 0.05,
  interior: 0.2,
  cinematic: 0.03,
  free: 0.1,
}

// Chase camera offsets relative to vehicle center (behind, above, look-ahead)
const CHASE_OFFSETS: Record<string, THREE.Vector3> = {
  exterior: new THREE.Vector3(0, 3, 8),
  'far-exterior': new THREE.Vector3(0, 5, 14),
  hood: new THREE.Vector3(0, 0.6, 1.5),
  grill: new THREE.Vector3(0, 0.3, 2.0),
  interior: new THREE.Vector3(0, 1.0, 0.3),
}

export function useCameraSystem(): CameraSystemHandle {
  const mode = ref<CameraMode>('exterior')
  const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 500)
  const currentPos = new THREE.Vector3()
  const currentLookAt = new THREE.Vector3()
  let disposed = false

  // Initialize to exterior defaults
  applyModeDefaults('exterior')

  function applyModeDefaults(m: CameraMode) {
    const defaults = CAMERA_DEFAULTS[m]
    if (defaults.fov !== undefined) camera.fov = defaults.fov
    if (defaults.near !== undefined) camera.near = defaults.near
    if (defaults.far !== undefined) camera.far = defaults.far
    camera.updateProjectionMatrix()
  }

  function setMode(m: CameraMode) {
    if (disposed || m === mode.value) return
    mode.value = m
    applyModeDefaults(m)
    logDebug('renderer:initialized', { cameraMode: m })
  }

  function update(targetPosition: THREE.Vector3, targetRotation: number, dt: number) {
    if (disposed) return
    const m = mode.value
    const smoothing = SMOOTHING[m]
    const alpha = 1 - Math.pow(1 - smoothing, dt / 16.67) // frame-rate independent smoothing

    let desiredPos = new THREE.Vector3()
    let desiredLookAt = targetPosition.clone()

    if (m === 'free') {
      // Free camera: don't move automatically, just look at target
      desiredPos.copy(currentPos)
    } else if (m === 'cinematic') {
      // Cinematic: orbit slowly around the vehicle
      const t = performance.now() * 0.0003
      desiredPos.set(
        targetPosition.x + Math.cos(t) * 12,
        targetPosition.y + 5,
        targetPosition.z + Math.sin(t) * 12,
      )
    } else {
      const offset = CHASE_OFFSETS[m] ?? CHASE_OFFSETS.exterior as THREE.Vector3
      // Rotate offset by vehicle yaw
      const cosR = Math.cos(targetRotation)
      const sinR = Math.sin(targetRotation)
      desiredPos.set(
        targetPosition.x + offset.x * cosR + offset.z * sinR,
        targetPosition.y + offset.y,
        targetPosition.z - offset.x * sinR + offset.z * cosR,
      )
    }

    currentPos.lerp(desiredPos, alpha)
    currentLookAt.lerp(desiredLookAt, alpha)

    camera.position.copy(currentPos)
    camera.lookAt(currentLookAt)
    camera.updateMatrixWorld()
  }

  function dispose() {
    if (disposed) return
    disposed = true
    logDebug('physics-worker:error', { message: 'camera:dispose' })
  }

  return {
    camera,
    currentMode: readonly(mode),
    setMode,
    update,
    dispose,
  }
}
