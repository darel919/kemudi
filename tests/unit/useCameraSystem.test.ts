import { describe, it, expect, vi } from 'vitest'
import * as THREE from 'three'

vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

import { useCameraSystem, type CameraMode } from '../../app/composables/useCameraSystem'

describe('useCameraSystem', () => {
  it('creates camera with default exterior mode', () => {
    const cs = useCameraSystem()
    expect(cs.camera).toBeInstanceOf(THREE.PerspectiveCamera)
    expect(cs.currentMode.value).toBe('exterior')
    cs.dispose()
  })

  it('setMode changes camera mode', () => {
    const cs = useCameraSystem()
    cs.setMode('hood')
    expect(cs.currentMode.value).toBe('hood')
    cs.setMode('grill')
    expect(cs.currentMode.value).toBe('grill')
    cs.dispose()
  })

  it('update moves camera toward target', () => {
    const cs = useCameraSystem()
    const target = new THREE.Vector3(5, 1, 5)
    cs.update(target, 0, 16.67)
    // Camera should have moved
    expect(cs.camera.position.length()).toBeGreaterThan(0)
    cs.dispose()
  })

  it('setMode is idempotent', () => {
    const cs = useCameraSystem()
    cs.setMode('exterior') // already exterior
    expect(cs.currentMode.value).toBe('exterior')
    cs.dispose()
  })

  it('dispose prevents further updates', () => {
    const cs = useCameraSystem()
    cs.dispose()
    // Should not throw
    cs.update(new THREE.Vector3(10, 10, 10), 0, 16.67)
  })

  it('all modes can be set without error', () => {
    const cs = useCameraSystem()
    const modes: CameraMode[] = [
      'exterior', 'hood', 'grill', 'far-exterior',
      'interior', 'cinematic', 'free',
    ]
    for (const m of modes) {
      cs.setMode(m)
      expect(cs.currentMode.value).toBe(m)
      cs.update(new THREE.Vector3(0, 0, 0), 0, 16.67)
    }
    cs.dispose()
  })
})
