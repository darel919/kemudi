import { describe, expect, it } from 'vitest'
import { getChartRange, interpolateCurve, toSvgPolyline } from '../../app/utils/telemetryChart'

describe('telemetry chart helpers', () => {
  it('interpolates the configured engine response curve', () => {
    expect(interpolateCurve([[1000, 120], [3000, 240], [7000, 180]], 2000)).toBe(180)
    expect(interpolateCurve([[1000, 120], [3000, 240]], 500)).toBe(120)
    expect(interpolateCurve([[1000, 120], [3000, 240]], 5000)).toBe(240)
  })

  it('ignores invalid points and returns a useful data range', () => {
    expect(interpolateCurve([[Number.NaN, 2], [1000, 100]], 1000)).toBe(100)
    expect(getChartRange([10, 20], { min: 0, max: 100 })).toEqual({ min: 0, max: 100 })
  })

  it('converts finite chart points to an SVG polyline', () => {
    expect(toSvgPolyline(
      [{ x: 0, y: 0 }, { x: 5, y: 5 }, { x: Number.NaN, y: 2 }],
      100,
      50,
      { min: 0, max: 5 },
      { min: 0, max: 5 },
    )).toBe('0.00,50.00 100.00,0.00')
  })
})
