import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import * as THREE from 'three'
import { useVehicleSkinning, computeWeights } from '../../app/composables/useVehicleSkinning'

// Stub logDebug to avoid import.meta.env.DEV issues
vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

beforeEach(() => {
  vi.clearAllMocks()
})

describe('computeWeights', () => {
  it('assigns weight 1 to exact-match node', () => {
    // 4 vertices at known positions, 2 nodes
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([
        0, 0, 0, // vertex 0 — on node 0
        1, 0, 0, // vertex 1 — on node 1
        0.5, 0, 0, // vertex 2 — equidistant
        2, 0, 0, // vertex 3 — closer to node 1
      ], 3),
    )

    const nodePositions = new Float64Array([
      0, 0, 0,  // node 0
      1, 0, 0,  // node 1
    ])

    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute
    const out = new Float32Array(4 * 2)
    computeWeights(out, posAttr, nodePositions, 4, 2)

    // Vertex 0 exactly on node 0 → weight [1, 0]
    expect(out[0]).toBeCloseTo(1)
    expect(out[1]).toBeCloseTo(0)

    // Vertex 1 exactly on node 1 → weight [0, 1]
    expect(out[2]).toBeCloseTo(0)
    expect(out[3]).toBeCloseTo(1)

    // Vertex 2 equidistant → weight [0.5, 0.5]
    expect(out[4]).toBeCloseTo(0.5)
    expect(out[5]).toBeCloseTo(0.5)

    // Vertex 3 at (2,0,0): dist to node0=2, dist to node1=1
    // w0 = (1/2) / (1/2 + 1) = 0.5/1.5 = 1/3
    // w1 = 1 / 1.5 = 2/3
    expect(out[6]).toBeCloseTo(1 / 3, 5)
    expect(out[7]).toBeCloseTo(2 / 3, 5)
  })

  it('weights sum to 1 for each vertex', () => {
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([
        1, 2, 3,
        4, 5, 6,
      ], 3),
    )

    const nodePositions = new Float64Array([
      0, 0, 0,
      10, 0, 0,
      0, 10, 0,
    ])

    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute
    const out = new Float32Array(2 * 3)
    computeWeights(out, posAttr, nodePositions, 2, 3)

    const sum0 = out[0]! + out[1]! + out[2]!
    const sum1 = out[3]! + out[4]! + out[5]!
    expect(sum0).toBeCloseTo(1, 5)
    expect(sum1).toBeCloseTo(1, 5)
  })
})

describe('useVehicleSkinning', () => {
  it('updates vertex positions based on node movement', () => {
    const geometry = new THREE.BufferGeometry()
    // Single vertex at origin, 2 nodes
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([0, 0, 0], 3),
    )

    const initialPositions = new Float64Array([
      -1, 0, 0,  // node 0
      +1, 0, 0,  // node 1
    ])

    const skinning = useVehicleSkinning(geometry, initialPositions)
    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute

    // Vertex at (0,0,0) is equidistant from both nodes
    // Weights should be [0.5, 0.5]
    // Move nodes: node0 → (-2,0,0), node1 → (2,0,0)
    // Expected vertex: 0.5*(-2) + 0.5*(2) = 0 in x → still 0
    skinning.update(new Float64Array([-2, 0, 0, 2, 0, 0]))
    expect(posAttr.array[0]).toBeCloseTo(0)
    expect(posAttr.array[1]).toBeCloseTo(0)
    expect(posAttr.array[2]).toBeCloseTo(0)

    // Shift both nodes right by 1
    skinning.update(new Float64Array([-1, 0, 0, 3, 0, 0]))
    expect(posAttr.array[0]).toBeCloseTo(1) // 0.5*(-1) + 0.5*3 = 1

    skinning.dispose()
  })

  it('marks position attribute dirty on update (version increments)', () => {
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([0, 0, 0], 3),
    )

    const initialPositions = new Float64Array([0, 0, 0])
    const skinning = useVehicleSkinning(geometry, initialPositions)
    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute

    // Three.js BufferAttribute.needsUpdate is a write-only setter that
    // increments `version`; it does not store the value. Verify via version.
    const versionBefore = posAttr.version
    skinning.update(new Float64Array([5, 5, 5]))
    expect(posAttr.version).toBeGreaterThan(versionBefore)

    skinning.dispose()
  })

  it('no-ops after dispose', () => {
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([1, 2, 3], 3),
    )

    const initialPositions = new Float64Array([0, 0, 0])
    const skinning = useVehicleSkinning(geometry, initialPositions)
    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute

    skinning.dispose()

    // Should not throw or modify positions
    skinning.update(new Float64Array([10, 20, 30]))
    expect(posAttr.array[0]).toBe(1)
    expect(posAttr.array[1]).toBe(2)
    expect(posAttr.array[2]).toBe(3)
  })

  it('is idempotent on dispose', () => {
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([0, 0, 0], 3),
    )

    const initialPositions = new Float64Array([0, 0, 0])
    const skinning = useVehicleSkinning(geometry, initialPositions)

    skinning.dispose()
    skinning.dispose() // should not throw
  })

  it('works with BoxGeometry (24 vertices, 8 nodes)', () => {
    const geometry = new THREE.BoxGeometry(1, 1, 1)
    const nodePositions = new Float64Array([
      -0.5, -0.5, +0.5,
      +0.5, -0.5, +0.5,
      -0.5, -0.5, -0.5,
      +0.5, -0.5, -0.5,
      -0.5, +0.5, +0.5,
      +0.5, +0.5, +0.5,
      -0.5, +0.5, -0.5,
      +0.5, +0.5, -0.5,
    ])

    const skinning = useVehicleSkinning(geometry, nodePositions)
    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute

    // Snapshot rest positions
    const before = Float32Array.from(posAttr.array as Float32Array)

    // Update with same positions → vertices should stay the same
    skinning.update(nodePositions)
    const after = Float32Array.from(posAttr.array as Float32Array)

    for (let i = 0; i < before.length; i++) {
      expect(after[i]).toBeCloseTo(before[i]!, 4)
    }

    skinning.dispose()
    geometry.dispose()
  })
})
