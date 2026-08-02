import { describe, expect, it } from 'vitest'
import { KNOWN_MAP_IDS, terrainProfileToId } from '../../app/types/base-map'

describe('base maps', () => {
  it('registers ground-zero, dragville, amazon', () => {
    expect(KNOWN_MAP_IDS).toEqual(['ground-zero', 'dragville', 'amazon'])
  })

  it('terrainProfileToId maps correctly', () => {
    expect(terrainProfileToId('ground-zero')).toBe(0)
    expect(terrainProfileToId('dragville')).toBe(1)
    expect(terrainProfileToId('amazon')).toBe(2)
  })
})
