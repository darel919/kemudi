import * as THREE from 'three'
import { logDebug } from '~/utils/debug'

export interface VehicleSkinningHandle {
  update(positions: Float64Array): void
  dispose(): void
}

export const CHASSIS_FRAME_SIZE = 12

/**
 * Build an orthonormal chassis frame from the first four suspension mounts.
 * Layout: origin, right, up, forward (three scalar components each).
 */
export function computeChassisFrame(out: Float64Array, positions: Float64Array): boolean {
  if (out.length < CHASSIS_FRAME_SIZE || positions.length < 12) return false
  const frontX = ((positions[0] ?? 0) + (positions[3] ?? 0)) * 0.5
  const frontY = ((positions[1] ?? 0) + (positions[4] ?? 0)) * 0.5
  const frontZ = ((positions[2] ?? 0) + (positions[5] ?? 0)) * 0.5
  const rearX = ((positions[6] ?? 0) + (positions[9] ?? 0)) * 0.5
  const rearY = ((positions[7] ?? 0) + (positions[10] ?? 0)) * 0.5
  const rearZ = ((positions[8] ?? 0) + (positions[11] ?? 0)) * 0.5
  const leftX = ((positions[0] ?? 0) + (positions[6] ?? 0)) * 0.5
  const leftY = ((positions[1] ?? 0) + (positions[7] ?? 0)) * 0.5
  const leftZ = ((positions[2] ?? 0) + (positions[8] ?? 0)) * 0.5
  const rightX = ((positions[3] ?? 0) + (positions[9] ?? 0)) * 0.5
  const rightY = ((positions[4] ?? 0) + (positions[10] ?? 0)) * 0.5
  const rightZ = ((positions[5] ?? 0) + (positions[11] ?? 0)) * 0.5

  let forwardX = frontX - rearX
  let forwardY = frontY - rearY
  let forwardZ = frontZ - rearZ
  const forwardLength = Math.hypot(forwardX, forwardY, forwardZ)
  if (!Number.isFinite(forwardLength) || forwardLength <= 1e-8) return false
  forwardX /= forwardLength
  forwardY /= forwardLength
  forwardZ /= forwardLength

  let basisRightX = rightX - leftX
  let basisRightY = rightY - leftY
  let basisRightZ = rightZ - leftZ
  const forwardProjection = basisRightX * forwardX + basisRightY * forwardY + basisRightZ * forwardZ
  basisRightX -= forwardX * forwardProjection
  basisRightY -= forwardY * forwardProjection
  basisRightZ -= forwardZ * forwardProjection
  const rightLength = Math.hypot(basisRightX, basisRightY, basisRightZ)
  if (!Number.isFinite(rightLength) || rightLength <= 1e-8) return false
  basisRightX /= rightLength
  basisRightY /= rightLength
  basisRightZ /= rightLength

  let upX = basisRightY * forwardZ - basisRightZ * forwardY
  let upY = basisRightZ * forwardX - basisRightX * forwardZ
  let upZ = basisRightX * forwardY - basisRightY * forwardX
  const upLength = Math.hypot(upX, upY, upZ)
  if (!Number.isFinite(upLength) || upLength <= 1e-8) return false
  upX /= upLength
  upY /= upLength
  upZ /= upLength

  out[0] = (frontX + rearX) * 0.5
  out[1] = (frontY + rearY) * 0.5
  out[2] = (frontZ + rearZ) * 0.5
  out[3] = basisRightX
  out[4] = basisRightY
  out[5] = basisRightZ
  out[6] = upX
  out[7] = upY
  out[8] = upZ
  out[9] = forwardX
  out[10] = forwardY
  out[11] = forwardZ
  return true
}

/** Align an authored body's horizontal footprint with its authoritative node cage. */
export function alignGeometryToChassisFootprint(
  geometry: THREE.BufferGeometry,
  nodePositions: Float64Array,
): void {
  if (nodePositions.length < 12) return
  geometry.computeBoundingBox()
  const bounds = geometry.boundingBox
  if (!bounds) return
  let minX = Number.POSITIVE_INFINITY
  let maxX = Number.NEGATIVE_INFINITY
  let minZ = Number.POSITIVE_INFINITY
  let maxZ = Number.NEGATIVE_INFINITY
  for (let index = 0; index < nodePositions.length; index += 3) {
    minX = Math.min(minX, nodePositions[index] ?? 0)
    maxX = Math.max(maxX, nodePositions[index] ?? 0)
    minZ = Math.min(minZ, nodePositions[index + 2] ?? 0)
    maxZ = Math.max(maxZ, nodePositions[index + 2] ?? 0)
  }
  const geometryWidth = bounds.max.x - bounds.min.x
  const geometryLength = bounds.max.z - bounds.min.z
  const chassisWidth = maxX - minX
  const chassisLength = maxZ - minZ
  if (geometryWidth <= 1e-6 || geometryLength <= 1e-6 || chassisWidth <= 1e-6 || chassisLength <= 1e-6) return
  const scaleX = chassisWidth / geometryWidth
  const scaleZ = chassisLength / geometryLength
  // Preserve an asset's authored overhang when it already agrees with the
  // cage. Large mismatches indicate that the mesh and physics definition are
  // in different horizontal scales and would visibly separate at the wheels.
  if (Math.abs(scaleX - 1) < 0.25 && Math.abs(scaleZ - 1) < 0.25) return
  const geometryCenterX = (bounds.min.x + bounds.max.x) * 0.5
  const geometryCenterZ = (bounds.min.z + bounds.max.z) * 0.5
  const chassisCenterX = (minX + maxX) * 0.5
  const chassisCenterZ = (minZ + maxZ) * 0.5
  geometry.translate(-geometryCenterX, 0, -geometryCenterZ)
  geometry.scale(scaleX, 1, scaleZ)
  geometry.translate(chassisCenterX, 0, chassisCenterZ)
  geometry.computeBoundingBox()
}

/**
 * Maps physics node positions to mesh vertex deformations via inverse-distance weighting.
 * Weight computation is done once at init; only position attributes are updated per frame.
 */
export function useVehicleSkinning(
  geometry: THREE.BufferGeometry,
  initialPositions: Float64Array,
): VehicleSkinningHandle {
  let disposed = false

  const nodeCount = initialPositions.length / 3
  const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute
  const vertexCount = posAttr.count
  const nodePositions = new Float64Array(initialPositions)
  // Keep the authored mesh shape and deform it by the weighted node
  // displacement. Replacing vertices with weighted node positions collapses
  // a GLB body into the physics cage at rest.
  const basePositions = new Float32Array(posAttr.array as Float32Array)
  const restFrame = new Float64Array(CHASSIS_FRAME_SIZE)
  const currentFrame = new Float64Array(CHASSIS_FRAME_SIZE)
  const hasRestFrame = computeChassisFrame(restFrame, initialPositions)
  const vertexLocalPositions = new Float32Array(vertexCount * 3)
  const nodeDeformation = new Float64Array(initialPositions.length)

  if (hasRestFrame) {
    for (let vi = 0; vi < vertexCount; vi++) {
      const vi3 = vi * posAttr.itemSize
      const dx = (basePositions[vi3] ?? 0) - (restFrame[0] ?? 0)
      const dy = (basePositions[vi3 + 1] ?? 0) - (restFrame[1] ?? 0)
      const dz = (basePositions[vi3 + 2] ?? 0) - (restFrame[2] ?? 0)
      const localOffset = vi * 3
      vertexLocalPositions[localOffset] = dx * (restFrame[3] ?? 0) + dy * (restFrame[4] ?? 0) + dz * (restFrame[5] ?? 0)
      vertexLocalPositions[localOffset + 1] = dx * (restFrame[6] ?? 0) + dy * (restFrame[7] ?? 0) + dz * (restFrame[8] ?? 0)
      vertexLocalPositions[localOffset + 2] = dx * (restFrame[9] ?? 0) + dy * (restFrame[10] ?? 0) + dz * (restFrame[11] ?? 0)
    }
  }

  // Precompute per-vertex, per-node weights: Float32Array[vertexCount * nodeCount]
  const weights = new Float32Array(vertexCount * nodeCount)
  computeWeights(weights, posAttr, initialPositions, vertexCount, nodeCount)

  logDebug('skinning:initialized', { vertexCount, nodeCount })

  function update(positions: Float64Array) {
    if (disposed) return
    if (positions.length !== nodePositions.length) return
    for (const position of positions) {
      if (!Number.isFinite(position)) return
    }
    nodePositions.set(positions)

    const posArray = posAttr.array as Float32Array
    const hasCurrentFrame = hasRestFrame && computeChassisFrame(currentFrame, positions)

    if (hasCurrentFrame) {
      // Decompose every node motion into the chassis' rigid transform plus a
      // residual soft-body deformation. Inverse-distance displacement alone
      // does not reproduce rotations, causing the shell to keep its old yaw
      // while the directly positioned wheels turn with the chassis.
      for (let ni = 0; ni < nodeCount; ni++) {
        const ni3 = ni * 3
        const restDx = (initialPositions[ni3] ?? 0) - (restFrame[0] ?? 0)
        const restDy = (initialPositions[ni3 + 1] ?? 0) - (restFrame[1] ?? 0)
        const restDz = (initialPositions[ni3 + 2] ?? 0) - (restFrame[2] ?? 0)
        const localX = restDx * (restFrame[3] ?? 0) + restDy * (restFrame[4] ?? 0) + restDz * (restFrame[5] ?? 0)
        const localY = restDx * (restFrame[6] ?? 0) + restDy * (restFrame[7] ?? 0) + restDz * (restFrame[8] ?? 0)
        const localZ = restDx * (restFrame[9] ?? 0) + restDy * (restFrame[10] ?? 0) + restDz * (restFrame[11] ?? 0)
        const rigidX = (currentFrame[0] ?? 0) + (currentFrame[3] ?? 0) * localX + (currentFrame[6] ?? 0) * localY + (currentFrame[9] ?? 0) * localZ
        const rigidY = (currentFrame[1] ?? 0) + (currentFrame[4] ?? 0) * localX + (currentFrame[7] ?? 0) * localY + (currentFrame[10] ?? 0) * localZ
        const rigidZ = (currentFrame[2] ?? 0) + (currentFrame[5] ?? 0) * localX + (currentFrame[8] ?? 0) * localY + (currentFrame[11] ?? 0) * localZ
        nodeDeformation[ni3] = (nodePositions[ni3] ?? 0) - rigidX
        nodeDeformation[ni3 + 1] = (nodePositions[ni3 + 1] ?? 0) - rigidY
        nodeDeformation[ni3 + 2] = (nodePositions[ni3 + 2] ?? 0) - rigidZ
      }
    }

    for (let vi = 0; vi < vertexCount; vi++) {
      const vi3 = vi * posAttr.itemSize
      const localOffset = vi * 3
      const localX = vertexLocalPositions[localOffset] ?? 0
      const localY = vertexLocalPositions[localOffset + 1] ?? 0
      const localZ = vertexLocalPositions[localOffset + 2] ?? 0
      let x = hasCurrentFrame
        ? (currentFrame[0] ?? 0) + (currentFrame[3] ?? 0) * localX + (currentFrame[6] ?? 0) * localY + (currentFrame[9] ?? 0) * localZ
        : (basePositions[vi3] ?? 0)
      let y = hasCurrentFrame
        ? (currentFrame[1] ?? 0) + (currentFrame[4] ?? 0) * localX + (currentFrame[7] ?? 0) * localY + (currentFrame[10] ?? 0) * localZ
        : (basePositions[vi3 + 1] ?? 0)
      let z = hasCurrentFrame
        ? (currentFrame[2] ?? 0) + (currentFrame[5] ?? 0) * localX + (currentFrame[8] ?? 0) * localY + (currentFrame[11] ?? 0) * localZ
        : (basePositions[vi3 + 2] ?? 0)
      const rowOffset = vi * nodeCount
      for (let ni = 0; ni < nodeCount; ni++) {
        const w = weights[rowOffset + ni] ?? 0
        if (w === 0) continue
        const ni3 = ni * 3
        x += w * (hasCurrentFrame
          ? (nodeDeformation[ni3] ?? 0)
          : ((nodePositions[ni3] ?? 0) - (initialPositions[ni3] ?? 0)))
        y += w * (hasCurrentFrame
          ? (nodeDeformation[ni3 + 1] ?? 0)
          : ((nodePositions[ni3 + 1] ?? 0) - (initialPositions[ni3 + 1] ?? 0)))
        z += w * (hasCurrentFrame
          ? (nodeDeformation[ni3 + 2] ?? 0)
          : ((nodePositions[ni3 + 2] ?? 0) - (initialPositions[ni3 + 2] ?? 0)))
      }
      posArray[vi3] = x
      posArray[vi3 + 1] = y
      posArray[vi3 + 2] = z
    }

    posAttr.needsUpdate = true
  }

  function dispose() {
    if (disposed) return
    disposed = true
    logDebug('skinning:disposed', { vertexCount, nodeCount })
  }

  return { update, dispose }
}

/** Compute normalized inverse-distance weights for each vertex relative to each node. */
export function computeWeights(
  out: Float32Array,
  posAttr: THREE.BufferAttribute,
  nodePositions: Float64Array,
  vertexCount: number,
  nodeCount: number,
  epsilon = 1e-6,
): void {
  const posArray = posAttr.array as Float32Array
  const stride = posAttr.itemSize
  const raw = new Float32Array(nodeCount)

  for (let vi = 0; vi < vertexCount; vi++) {
    const vi3 = vi * stride
    const vx = posArray[vi3] ?? 0
    const vy = posArray[vi3 + 1] ?? 0
    const vz = posArray[vi3 + 2] ?? 0
    let totalW = 0
    let exactMatch = -1

    for (let ni = 0; ni < nodeCount; ni++) {
      const ni3 = ni * 3
      const dx = vx - (nodePositions[ni3] ?? 0)
      const dy = vy - (nodePositions[ni3 + 1] ?? 0)
      const dz = vz - (nodePositions[ni3 + 2] ?? 0)
      const dist2 = dx * dx + dy * dy + dz * dz

      if (dist2 < epsilon * epsilon) {
        exactMatch = ni
        raw[ni] = 0
      } else {
        const dist = Math.sqrt(dist2)
        const w = 1 / dist
        raw[ni] = w
        totalW += w
      }
    }

    const rowOffset = vi * nodeCount
    if (exactMatch >= 0) {
      for (let ni = 0; ni < nodeCount; ni++) {
        out[rowOffset + ni] = 0
      }
      out[rowOffset + exactMatch] = 1
    } else {
      const invTotal = totalW > 0 ? 1 / totalW : 0
      for (let ni = 0; ni < nodeCount; ni++) {
        out[rowOffset + ni] = (raw[ni] ?? 0) * invTotal
      }
    }
  }
}
