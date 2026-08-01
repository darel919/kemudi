import * as THREE from 'three'
import { logDebug } from '~/utils/debug'

export interface MaterialLibrary {
  body: THREE.MeshStandardMaterial
  glass: THREE.MeshStandardMaterial
  metal: THREE.MeshStandardMaterial
  plastic: THREE.MeshStandardMaterial
  rubber: THREE.MeshStandardMaterial
  dispose(): void
}

/**
 * ponytail: only basic PBR materials; add normal maps, roughness maps,
 * and damage overlays when texture assets are available.
 */
export function createMaterialLibrary(): MaterialLibrary {
  const body = new THREE.MeshStandardMaterial({
    color: 0xcc3333,
    roughness: 0.35,
    metalness: 0.7,
  })

  const glass = new THREE.MeshStandardMaterial({
    color: 0xaaddff,
    roughness: 0.05,
    metalness: 0.0,
    transparent: true,
    opacity: 0.35,
  })

  const metal = new THREE.MeshStandardMaterial({
    color: 0x888888,
    roughness: 0.25,
    metalness: 0.9,
  })

  const plastic = new THREE.MeshStandardMaterial({
    color: 0x333333,
    roughness: 0.6,
    metalness: 0.0,
  })

  const rubber = new THREE.MeshStandardMaterial({
    color: 0x222222,
    roughness: 0.9,
    metalness: 0.0,
  })

  logDebug('renderer:initialized', { component: 'MaterialLibrary' })

  return {
    body, glass, metal, plastic, rubber,
    dispose() {
      body.dispose()
      glass.dispose()
      metal.dispose()
      plastic.dispose()
      rubber.dispose()
    },
  }
}

// ===== Particle System =====

export interface ParticleSystem {
  /** Update particle positions; call once per frame */
  update(dt: number): void
  /** Get the Three.js points object to add to the scene */
  getObject(): THREE.Points
  /** Emit N particles at a position with velocity spread */
  emit(count: number, position: THREE.Vector3, velocity: THREE.Vector3, spread: number): void
  dispose(): void
}

interface Particle {
  position: THREE.Vector3
  velocity: THREE.Vector3
  life: number
  maxLife: number
  size: number
}

const MAX_PARTICLES = 500

export function createParticleSystem(budget = MAX_PARTICLES): ParticleSystem {
  const particles: Particle[] = []
  const positions = new Float32Array(budget * 3)
  const sizes = new Float32Array(budget)

  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1))

  const material = new THREE.PointsMaterial({
    color: 0xbbbbbb,
    size: 0.3,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.6,
    depthWrite: false,
  })

  const points = new THREE.Points(geometry, material)
  points.frustumCulled = false

  function emit(count: number, position: THREE.Vector3, velocity: THREE.Vector3, spread: number) {
    for (let i = 0; i < count && particles.length < budget; i++) {
      particles.push({
        position: position.clone(),
        velocity: new THREE.Vector3(
          velocity.x + (Math.random() - 0.5) * spread,
          velocity.y + (Math.random() - 0.5) * spread,
          velocity.z + (Math.random() - 0.5) * spread,
        ),
        life: 0,
        maxLife: 1.0 + Math.random() * 2.0,
        size: 0.1 + Math.random() * 0.4,
      })
    }
  }

  function update(dt: number) {
    const dtSec = dt / 1000

    // Update and cull dead particles
    for (let i = particles.length - 1; i >= 0; i--) {
      const p = particles[i]!
      p.life += dtSec
      if (p.life >= p.maxLife) {
        particles.splice(i, 1)
        continue
      }
      p.position.addScaledVector(p.velocity, dtSec)
      p.velocity.y -= 2.0 * dtSec // gravity
      p.velocity.multiplyScalar(0.98) // drag
    }

    // Write to buffers
    for (let i = 0; i < budget; i++) {
      if (i < particles.length) {
        const p = particles[i]!
        const fade = 1 - p.life / p.maxLife
        positions[i * 3 + 0] = p.position.x
        positions[i * 3 + 1] = p.position.y
        positions[i * 3 + 2] = p.position.z
        sizes[i] = p.size * fade
      } else {
        positions[i * 3 + 1] = -9999 // hide unused
        sizes[i] = 0
      }
    }

    ;(geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true
    ;(geometry.getAttribute('size') as THREE.BufferAttribute).needsUpdate = true
  }

  function getObject() { return points }

  function dispose() {
    geometry.dispose()
    material.dispose()
    particles.length = 0
  }

  return { update, getObject, emit, dispose }
}

// ===== Skid Mark Renderer =====

export interface SkidMarks {
  /** Add a skid point */
  addPoint(position: THREE.Vector3, normal: THREE.Vector3): void
  /** Get the Three.js mesh to add to the scene */
  getObject(): THREE.Mesh
  dispose(): void
}

const MAX_SKID_POINTS = 2000

export function createSkidMarks(): SkidMarks {
  const positions = new Float32Array(MAX_SKID_POINTS * 6) // 2 vertices per point (quad strip)
  const normals = new Float32Array(MAX_SKID_POINTS * 6)

  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3))

  const material = new THREE.MeshStandardMaterial({
    color: 0x111111,
    roughness: 0.95,
    metalness: 0.0,
    depthWrite: false,
  })

  const mesh = new THREE.Mesh(geometry, material)
  mesh.frustumCulled = false
  mesh.renderOrder = -1

  let writeIndex = 0
  let pointCount = 0

  function addPoint(position: THREE.Vector3, normal: THREE.Vector3) {
    if (pointCount >= MAX_SKID_POINTS / 2) return

    const i = writeIndex % MAX_SKID_POINTS
    const w = 0.08 // half-width of skid mark

    // Two vertices forming a small quad segment
    const right = new THREE.Vector3().crossVectors(normal, new THREE.Vector3(0, 0, 1)).normalize()
    if (right.lengthSq() < 0.01) right.set(1, 0, 0)

    const p1 = position.clone().addScaledVector(right, -w)
    const p2 = position.clone().addScaledVector(right, w)

    positions[i * 6 + 0] = p1.x; positions[i * 6 + 1] = p1.y; positions[i * 6 + 2] = p1.z
    positions[i * 6 + 3] = p2.x; positions[i * 6 + 4] = p2.y; positions[i * 6 + 5] = p2.z

    normals[i * 6 + 0] = normal.x; normals[i * 6 + 1] = normal.y; normals[i * 6 + 2] = normal.z
    normals[i * 6 + 3] = normal.x; normals[i * 6 + 4] = normal.y; normals[i * 6 + 5] = normal.z

    writeIndex++
    pointCount = Math.min(pointCount + 1, MAX_SKID_POINTS / 2)

    geometry.setDrawRange(0, pointCount * 2)
    ;(geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true
    ;(geometry.getAttribute('normal') as THREE.BufferAttribute).needsUpdate = true
  }

  function getObject() { return mesh }

  function dispose() {
    geometry.dispose()
    material.dispose()
  }

  return { addPoint, getObject, dispose }
}
