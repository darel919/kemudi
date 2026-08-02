import { describe, expect, it } from 'vitest'
import { normalizeHeightmapPixels } from '../../app/utils/heightmap'

describe('heightmap decoding', () => {
  it('normalizes grayscale pixels to bounded elevation samples', () => {
    expect(normalizeHeightmapPixels(new Uint8ClampedArray([0, 127, 255]), 3, 2)).toEqual(new Float32Array([-2, 0, 2]))
  })

  it('rejects malformed pixel buffers and invalid scale', () => {
    expect(() => normalizeHeightmapPixels(new Uint8ClampedArray([0, 1]), 3, 2)).toThrow(/pixel/i)
    expect(() => normalizeHeightmapPixels(new Uint8ClampedArray([0]), 1, Infinity)).toThrow(/scale/i)
  })
})
