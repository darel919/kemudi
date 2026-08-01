import * as THREE from 'three'
import { logDebug } from '~/utils/debug'

export interface VehicleSkinningHandle {
  update(positions: Float64Array): void
  dispose(): void
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

    for (let vi = 0; vi < vertexCount; vi++) {
      const vi3 = vi * posAttr.itemSize
      let x = basePositions[vi3] ?? 0
      let y = basePositions[vi3 + 1] ?? 0
      let z = basePositions[vi3 + 2] ?? 0
      const rowOffset = vi * nodeCount
      for (let ni = 0; ni < nodeCount; ni++) {
        const w = weights[rowOffset + ni] ?? 0
        if (w === 0) continue
        const ni3 = ni * 3
        x += w * ((nodePositions[ni3] ?? 0) - (initialPositions[ni3] ?? 0))
        y += w * ((nodePositions[ni3 + 1] ?? 0) - (initialPositions[ni3 + 1] ?? 0))
        z += w * ((nodePositions[ni3 + 2] ?? 0) - (initialPositions[ni3 + 2] ?? 0))
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
