import * as THREE from 'three'

/** Match the Rust common-turn-center Ackermann calculation without allocations. */
export function computeVisualAckermannAngles(
  out: Float64Array,
  steeringAngle: number,
  wheelbase: number,
  trackWidth: number,
): void {
  if (out.length < 2) return
  if (!Number.isFinite(steeringAngle) || Math.abs(steeringAngle) < 1e-6) {
    out[0] = 0
    out[1] = 0
    return
  }

  const sign = steeringAngle > 0 ? 1 : -1
  const absoluteAngle = Math.min(Math.abs(steeringAngle), Math.PI * 0.5 - 1e-4)
  const safeWheelbase = Math.max(0.1, wheelbase)
  const halfTrack = Math.max(0.1, trackWidth) * 0.5
  const centerRadius = safeWheelbase / Math.max(1e-6, Math.tan(absoluteAngle))
  const inner = sign * Math.atan(safeWheelbase / Math.max(0.05, centerRadius - halfTrack))
  const outer = sign * Math.atan(safeWheelbase / (centerRadius + halfTrack))

  if (steeringAngle > 0) {
    out[0] = outer
    out[1] = inner
  } else {
    out[0] = inner
    out[1] = outer
  }
}

/**
 * Apply the chassis orientation to every wheel, then add steering only to the
 * front axle. The authored vehicle faces local -Z, hence the visual sign.
 */
export function applyWheelOrientation(
  wheelPivot: THREE.Object3D,
  chassisOrientation: THREE.Quaternion,
  wheelIndex: number,
  frontSteeringAngles: Float64Array,
): void {
  wheelPivot.quaternion.copy(chassisOrientation)
  if (wheelIndex < 2) {
    wheelPivot.rotateY(-(frontSteeringAngles[wheelIndex] ?? 0))
  }
}
