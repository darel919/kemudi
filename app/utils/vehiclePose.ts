/**
 * Estimate vehicle yaw from a deformable chassis while preserving the
 * authored front/rear node groups. Averaging those groups avoids making the
 * ADAS sensor corridor follow a single bent suspension corner.
 */
export function getVehicleYaw(
  positions: ArrayLike<number>,
  referencePositions: ArrayLike<number>,
): number {
  if (positions.length < 12 || referencePositions.length !== positions.length) return 0

  let minReferenceZ = Number.POSITIVE_INFINITY
  let maxReferenceZ = Number.NEGATIVE_INFINITY
  for (let index = 2; index < referencePositions.length; index += 3) {
    const referenceZ = referencePositions[index] ?? 0
    minReferenceZ = Math.min(minReferenceZ, referenceZ)
    maxReferenceZ = Math.max(maxReferenceZ, referenceZ)
  }
  const span = maxReferenceZ - minReferenceZ
  if (!Number.isFinite(span) || span < 0.001) return 0

  const endDepth = Math.max(0.25, span * 0.35)
  const frontLimit = minReferenceZ + endDepth
  const rearLimit = maxReferenceZ - endDepth
  let frontX = 0
  let frontZ = 0
  let frontCount = 0
  let rearX = 0
  let rearZ = 0
  let rearCount = 0

  for (let index = 2; index < positions.length; index += 3) {
    const referenceZ = referencePositions[index] ?? 0
    const currentX = positions[index - 2] ?? 0
    const currentZ = positions[index] ?? 0
    if (referenceZ <= frontLimit) {
      frontX += currentX
      frontZ += currentZ
      frontCount++
    } else if (referenceZ >= rearLimit) {
      rearX += currentX
      rearZ += currentZ
      rearCount++
    }
  }

  if (!frontCount || !rearCount) return 0
  const frontMeanX = frontX / frontCount
  const frontMeanZ = frontZ / frontCount
  const rearMeanX = rearX / rearCount
  const rearMeanZ = rearZ / rearCount
  return Math.atan2(frontMeanX - rearMeanX, rearMeanZ - frontMeanZ)
}
