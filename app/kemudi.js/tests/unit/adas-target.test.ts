import { describe, expect, it } from 'vitest'
import { findAdasTarget } from '~/utils/adasTarget'
import { getVehicleYaw } from '~/utils/vehiclePose'
import type { MapPhysicsData } from '~/types/physics'

function emptyMap(): MapPhysicsData {
  return { primitives: [], boundaries: [], roads: [] }
}

describe('ADAS target selection', () => {
  it('detects a static wall in the vehicle path with a closing relative speed', () => {
    const map = emptyMap()
    map.boundaries.push({
      kind: 'box',
      center: { x: 0, y: 1, z: -5 },
      halfExtents: { x: 3, y: 1, z: 0.5 },
      restitution: 0.1,
      friction: 0.8,
    })

    const target = findAdasTarget(
      { x: 0, z: 0 },
      0,
      10,
      map,
      [],
    )

    expect(target.distance).toBeCloseTo(4.5)
    expect(target.relativeSpeed).toBeCloseTo(-10)
  })

  it('ignores an obstacle outside the forward path and selects the nearer closing target', () => {
    const map = emptyMap()
    map.primitives.push({
      kind: 'sphere',
      center: { x: 5, y: 1, z: -4 },
      radius: 0.5,
      restitution: 0.1,
      friction: 0.8,
    })
    map.primitives.push({
      kind: 'box',
      center: { x: 0, y: 1, z: -8 },
      halfExtents: { x: 1, y: 1, z: 0.5 },
      restitution: 0.1,
      friction: 0.8,
    })

    const target = findAdasTarget(
      { x: 0, z: 0 },
      0,
      10,
      map,
      [{ positions: new Float64Array([0, 0, -3]), speedMps: 3 }],
    )

    expect(target.distance).toBeCloseTo(3)
    expect(target.relativeSpeed).toBeCloseTo(-7)
  })

  it('keeps the ADAS heading stable when one front chassis node is deformed', () => {
    const reference = new Float64Array([
      -0.8, 0, -1.35, 0.8, 0, -1.35,
      -0.65, 0, -0.75, 0.65, 0, -0.75,
      -0.65, 0, 0.75, 0.65, 0, 0.75,
      -0.8, 0, 1.35, 0.8, 0, 1.35,
    ])
    const yaw = 0.6
    const current = new Float64Array(reference)
    for (let index = 0; index < reference.length; index += 3) {
      const x = reference[index] ?? 0
      const z = reference[index + 2] ?? 0
      current[index] = Math.cos(yaw) * x - Math.sin(yaw) * z
      current[index + 2] = Math.sin(yaw) * x + Math.cos(yaw) * z
    }
    current[0] += 0.5
    current[2] -= 0.35

    expect(getVehicleYaw(current, reference)).toBeCloseTo(yaw, 1)
  })
})
