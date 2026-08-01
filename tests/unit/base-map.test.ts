import { describe, expect, it } from 'vitest'
import { BASE_MAPS, getBaseMap } from '../../app/types/base-map'

describe('base maps', () => {
  it('provides flat, bumpy, and offroad levels', () => {
    expect(Object.keys(BASE_MAPS)).toEqual(['flat', 'bumpy', 'offroad'])
    expect(getBaseMap('flat').heightScale).toBe(0)
    expect(getBaseMap('bumpy').profile).toBe('bumpy')
    expect(getBaseMap('offroad').surfaceLabel).toBe('LOOSE DIRT')
  })
})
