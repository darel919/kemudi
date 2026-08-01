import { describe, it, expect, vi } from 'vitest'
import * as THREE from 'three'

vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

import { createMaterialLibrary, createParticleSystem, createSkidMarks } from '../../app/utils/materials'
import { useTerrain } from '../../app/composables/useTerrain'

describe('MaterialLibrary', () => {
  it('creates all material types', () => {
    const lib = createMaterialLibrary()
    expect(lib.body).toBeInstanceOf(THREE.MeshStandardMaterial)
    expect(lib.glass).toBeInstanceOf(THREE.MeshStandardMaterial)
    expect(lib.metal).toBeInstanceOf(THREE.MeshStandardMaterial)
    expect(lib.plastic).toBeInstanceOf(THREE.MeshStandardMaterial)
    expect(lib.rubber).toBeInstanceOf(THREE.MeshStandardMaterial)
    lib.dispose()
  })

  it('glass is transparent', () => {
    const lib = createMaterialLibrary()
    expect(lib.glass.transparent).toBe(true)
    expect(lib.glass.opacity).toBeLessThan(1)
    lib.dispose()
  })

  it('dispose cleans up all materials', () => {
    const lib = createMaterialLibrary()
    // dispose should not throw
    lib.dispose()
  })
})

describe('ParticleSystem', () => {
  it('creates with default budget', () => {
    const ps = createParticleSystem()
    expect(ps.getObject()).toBeInstanceOf(THREE.Points)
    ps.dispose()
  })

  it('emits and updates particles', () => {
    const ps = createParticleSystem(100)
    const pos = new THREE.Vector3(0, 1, 0)
    const vel = new THREE.Vector3(0, 2, 0)
    ps.emit(5, pos, vel, 0.5)
    ps.update(16) // 16ms frame
    ps.dispose()
  })

  it('dispose is idempotent', () => {
    const ps = createParticleSystem()
    ps.dispose()
    ps.dispose()
  })
})

describe('SkidMarks', () => {
  it('creates skid mark renderer', () => {
    const sm = createSkidMarks()
    expect(sm.getObject()).toBeInstanceOf(THREE.Mesh)
    sm.dispose()
  })

  it('adds points without error', () => {
    const sm = createSkidMarks()
    sm.addPoint(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 1, 0))
    sm.addPoint(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0))
    sm.dispose()
  })
})

describe('Terrain', () => {
  it('creates terrain mesh', () => {
    const terrain = useTerrain({ segmentsW: 16, segmentsD: 16 })
    expect(terrain.mesh).toBeInstanceOf(THREE.Mesh)
    expect(terrain.geometry).toBeInstanceOf(THREE.PlaneGeometry)
    terrain.dispose()
  })

  it('returns valid height values', () => {
    const terrain = useTerrain({ width: 100, depth: 100, segmentsW: 8, segmentsD: 8 })
    const h = terrain.getHeightAt(0, 0)
    expect(typeof h).toBe('number')
    expect(Number.isFinite(h)).toBe(true)
    terrain.dispose()
  })

  it('returns valid normal', () => {
    const terrain = useTerrain({ segmentsW: 8, segmentsD: 8 })
    const n = terrain.getNormalAt(0, 0)
    expect(n).toBeInstanceOf(THREE.Vector3)
    expect(n.length()).toBeCloseTo(1.0, 5)
    terrain.dispose()
  })

  it('dispose cleans up resources', () => {
    const terrain = useTerrain({ segmentsW: 4, segmentsD: 4 })
    // dispose should not throw
    terrain.dispose()
  })
})
