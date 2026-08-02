export interface ChartPoint {
  x: number
  y: number
}

export interface ChartRange {
  min: number
  max: number
}

export function interpolateCurve(curve: readonly (readonly [number, number])[], x: number): number | null {
  const points = curve
    .filter(point => Number.isFinite(point[0]) && Number.isFinite(point[1]))
    .sort((a, b) => a[0] - b[0])

  if (!points.length || !Number.isFinite(x)) return null
  if (x <= points[0]![0]) return points[0]![1]
  if (x >= points[points.length - 1]![0]) return points[points.length - 1]![1]

  for (let index = 1; index < points.length; index++) {
    const previous = points[index - 1]!
    const current = points[index]!
    if (x > current[0]) continue
    const span = current[0] - previous[0]
    if (span <= 0) return current[1]
    const progress = (x - previous[0]) / span
    return previous[1] + (current[1] - previous[1]) * progress
  }

  return null
}

export function getChartRange(values: readonly number[], fallback: ChartRange, padding = 0.08): ChartRange {
  const finiteValues = values.filter(Number.isFinite)
  if (!finiteValues.length) return fallback

  const dataMin = Math.min(...finiteValues)
  const dataMax = Math.max(...finiteValues)
  const span = Math.max(dataMax - dataMin, Math.abs(dataMax) * 0.1, 1)
  const min = Math.min(fallback.min, dataMin - span * padding)
  const max = Math.max(fallback.max, dataMax + span * padding)
  return max > min ? { min, max } : { min: min - 1, max: max + 1 }
}

export function toSvgPolyline(
  points: readonly ChartPoint[],
  width: number,
  height: number,
  xRange: ChartRange,
  yRange: ChartRange,
): string {
  const xSpan = Math.max(xRange.max - xRange.min, Number.EPSILON)
  const ySpan = Math.max(yRange.max - yRange.min, Number.EPSILON)

  return points
    .filter(point => Number.isFinite(point.x) && Number.isFinite(point.y))
    .map((point) => {
      const x = ((point.x - xRange.min) / xSpan) * width
      const y = height - ((point.y - yRange.min) / ySpan) * height
      return `${x.toFixed(2)},${y.toFixed(2)}`
    })
    .join(' ')
}
