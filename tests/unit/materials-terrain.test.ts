import { describe, it, expect, vi } from 'vitest'
import * as THREE from 'three'

vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

import { createMaterialLibrary, createParticleSystem, createSkidMarks } from '../../app/utils/materials'
import { useTerrainFromMap as useTerrain } from '../../app/composables/useTerrain'
import type { MapDefinition } from '../../app/types/map-schema'

const testMap: MapDefinition = {
  id: 'ground-zero',
  label: 'Test',
  description: '',
  version: 1,
  size: { width: 100, depth: 100 },
  segments: 16,
  preview: { bgColor: 0x3b4650, pattern: 'grid' },
  terrain: {
    heightmap: null, heightScale: 0, color: 0x3b4650, roughness: 0.76,
    groundFriction: 0.94, procedural: null,
    layers: [{ name: 'Asphalt', surfaceId: 0, color: 0x3b4650, slopeRange: [0, 90], heightRange: [0, 1000], priority: 0 }],
  },
  roads: [], objects: [], spawnPoints: [],
}

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
    ps.update(16)
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
  it('creates terrain mesh from MapDefinition', () => {
    const terrain = useTerrain(testMap)
    expect(terrain.mesh).toBeInstanceOf(THREE.Mesh)
    expect(terrain.geometry).toBeInstanceOf(THREE.PlaneGeometry)
    terrain.dispose()
  })

  it('returns valid normal', () => {
    const terrain = useTerrain(testMap)
    const n = terrain.getNormalAt(0, 0)
    expect(n).toBeInstanceOf(THREE.Vector3)
    expect(n.length()).toBeCloseTo(1.0, 5)
    terrain.dispose()
  })

  it('dispose cleans up resources', () => {
    const terrain = useTerrain(testMap)
    terrain.dispose()
  })

  it('exposes authoritative collision primitives for objects and map boundaries', () => {
    const terrain = useTerrain({
      ...testMap,
      objects: [{ type: 'box', placement: { mode: 'instance', position: { x: 2, z: 3 } }, visual: { size: [2, 4, 6] } }],
      boundaries: [{ position: { x: 0, y: 1, z: -50 }, size: [100, 2, 1] }],
    })
    expect(terrain.collisionData.primitives).toHaveLength(1)
    expect(terrain.collisionData.primitives[0]).toMatchObject({ kind: 'box', center: { x: 2, z: 3 }, halfExtents: { x: 1, y: 2, z: 3 } })
    expect(terrain.collisionData.boundaries).toHaveLength(1)
    terrain.dispose()
  })
})
