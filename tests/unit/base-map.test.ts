import { describe, expect, it } from 'vitest'
import { KNOWN_MAP_IDS, mapToSurfacePresetIndex, terrainProfileToId, validateMap } from '../../app/types/base-map'

describe('base maps', () => {
  it('registers ground-zero, dragville, amazon', () => {
    expect(KNOWN_MAP_IDS).toEqual(['ground-zero', 'dragville', 'amazon'])
  })

  it('terrainProfileToId maps correctly', () => {
    expect(terrainProfileToId('ground-zero')).toBe(0)
    expect(terrainProfileToId('dragville')).toBe(1)
    expect(terrainProfileToId('amazon')).toBe(2)
  })

  it('maps the lowest-priority terrain layer to the worker surface preset', () => {
    const map = validateMap({
      id: 'amazon',
      size: { width: 100, depth: 80 },
      segments: 16,
      terrain: {
        heightmap: null,
        heightScale: 14,
        color: 1,
        roughness: 0.95,
        groundFriction: 0.72,
        layers: [
          { name: 'Mud', surfaceId: 6, slopeRange: [0, 15], heightRange: [0, 3], priority: 2 },
          { name: 'Jungle Floor', surfaceId: 3, slopeRange: [0, 30], heightRange: [0, 1000], priority: 0 },
        ],
      },
      roads: [],
      objects: [],
      spawnPoints: [{ x: 0, z: 0, heading: 0 }],
    })
    expect(mapToSurfacePresetIndex(map)).toBe(3)
  })

  it('normalizes collision boundaries and road surface metadata', () => {
    const map = validateMap({
      id: 'ground-zero',
      size: { width: 100, depth: 80 },
      segments: 16,
      terrain: { heightmap: null, heightScale: 0, color: 1, roughness: 0.5, groundFriction: 0.9, layers: [] },
      surfaces: [{ id: 4, name: 'wet-gravel', friction: 0.55, roughness: 0.8, moisture: 0.6, compactness: 0.4 }],
      roads: [{ name: 'loop', width: 8, surfaceId: 4, surface: { friction: 0.55, roughness: 0.8, moisture: 0.6, compactness: 0.4 }, points: [{ x: 0, z: 0 }, { x: 10, z: 0 }], visible: true }],
      objects: [{ type: 'sphere', placement: { mode: 'instance', position: { x: 1, z: 2 } }, visual: { radius: 2 } }],
      boundaries: [{ name: 'north', position: { x: 0, y: 1, z: -40 }, size: [100, 2, 1] }],
      spawnPoints: [{ x: 0, z: 0, heading: 0 }],
    })
    expect(map.boundaries).toHaveLength(1)
    expect(map.objects[0]?.collision).toEqual({ type: 'sphere', radius: 2 })
    expect(map.roads[0]?.surface?.friction).toBe(0.55)
  })

  it('rejects non-finite values and unknown road references', () => {
    expect(() => validateMap({ id: 'ground-zero', size: { width: Infinity, depth: 10 }, terrain: {} })).toThrow(/finite/i)
    expect(() => validateMap({
      id: 'ground-zero', size: { width: 10, depth: 10 }, terrain: { layers: [] },
      roads: [{ name: 'loop', width: 2, surfaceId: 0, points: [{ x: 0, z: 0 }], visible: true }],
      objects: [{ type: 'box', placement: { mode: 'spline', roadName: 'missing', spacing: 1, offset: 0 }, visual: { size: [1, 1, 1] } }],
    })).toThrow(/unknown road/i)
  })
})
