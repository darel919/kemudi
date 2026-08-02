import type { MapCollisionBox, MapCollisionSphere, MapPhysicsData } from '~/types/physics'

export interface AdasTarget {
  distance: number
  relativeSpeed: number
}

export interface AdasRemoteTarget {
  positions: ArrayLike<number>
  /** Forward speed in m/s. `telemetry[0]` is accepted for worker snapshots. */
  speedMps?: number
  telemetry?: ArrayLike<number>
}

interface Point2D {
  x: number
  z: number
}

const SENSOR_LATERAL_HALF_WIDTH = 2.5

/**
 * Select the closest object in the forward sensor corridor.
 *
 * The Rust ADAS controller uses a negative target-relative speed for a
 * closing target. Static map geometry therefore reports `-egoSpeedMps` and
 * remote vehicles report `targetSpeedMps - egoSpeedMps`.
 */
export function findAdasTarget(
  center: Point2D,
  yaw: number,
  egoSpeedMps: number,
  map: Pick<MapPhysicsData, 'primitives' | 'boundaries'> | undefined,
  remoteTargets: readonly AdasRemoteTarget[],
): AdasTarget {
  const safeYaw = Number.isFinite(yaw) ? yaw : 0
  const safeEgoSpeed = Number.isFinite(egoSpeedMps) ? Math.max(0, egoSpeedMps) : 0
  const forwardX = Math.sin(safeYaw)
  const forwardZ = -Math.cos(safeYaw)
  const rightX = Math.cos(safeYaw)
  const rightZ = Math.sin(safeYaw)
  let closest = Number.POSITIVE_INFINITY
  let relativeSpeed = 0

  const consider = (distance: number, targetRelativeSpeed: number) => {
    if (!Number.isFinite(distance) || distance <= 0 || distance >= closest) return
    closest = distance
    relativeSpeed = Number.isFinite(targetRelativeSpeed) ? targetRelativeSpeed : 0
  }

  const considerBox = (box: MapCollisionBox) => {
    const dx = box.center.x - center.x
    const dz = box.center.z - center.z
    const forwardDistance = dx * forwardX + dz * forwardZ
    const lateralDistance = dx * rightX + dz * rightZ
    const forwardExtent = Math.abs(forwardX) * Math.abs(box.halfExtents.x)
      + Math.abs(forwardZ) * Math.abs(box.halfExtents.z)
    const lateralExtent = Math.abs(rightX) * Math.abs(box.halfExtents.x)
      + Math.abs(rightZ) * Math.abs(box.halfExtents.z)
    if (forwardDistance + forwardExtent <= 0) return
    if (Math.abs(lateralDistance) > SENSOR_LATERAL_HALF_WIDTH + lateralExtent) return
    consider(Math.max(0.01, forwardDistance - forwardExtent), -safeEgoSpeed)
  }

  const considerSphere = (sphere: MapCollisionSphere) => {
    const dx = sphere.center.x - center.x
    const dz = sphere.center.z - center.z
    const forwardDistance = dx * forwardX + dz * forwardZ
    const lateralDistance = dx * rightX + dz * rightZ
    const radius = Math.abs(sphere.radius)
    if (forwardDistance + radius <= 0) return
    if (Math.abs(lateralDistance) > SENSOR_LATERAL_HALF_WIDTH + radius) return
    consider(Math.max(0.01, forwardDistance - radius), -safeEgoSpeed)
  }

  for (const primitive of map?.primitives ?? []) {
    if (primitive.kind === 'box') considerBox(primitive)
    else considerSphere(primitive)
  }
  for (const boundary of map?.boundaries ?? []) considerBox(boundary)

  for (const remote of remoteTargets) {
    if (remote.positions.length < 3) continue
    let x = 0
    let z = 0
    let count = 0
    for (let index = 0; index + 2 < remote.positions.length; index += 3) {
      x += remote.positions[index] ?? 0
      z += remote.positions[index + 2] ?? 0
      count++
    }
    if (!count) continue
    const dx = x / count - center.x
    const dz = z / count - center.z
    const forwardDistance = dx * forwardX + dz * forwardZ
    const lateralDistance = dx * rightX + dz * rightZ
    if (forwardDistance <= 0 || Math.abs(lateralDistance) > SENSOR_LATERAL_HALF_WIDTH) continue
    const distance = Math.sqrt(dx * dx + dz * dz)
    const targetSpeed = remote.speedMps ?? remote.telemetry?.[0] ?? 0
    consider(distance, targetSpeed - safeEgoSpeed)
  }

  return Number.isFinite(closest)
    ? { distance: closest, relativeSpeed }
    : { distance: 0, relativeSpeed: 0 }
}
