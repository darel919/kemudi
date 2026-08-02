/**
 * Generic terrain renderer.
 *
 * Reads a MapDefinition and creates Three.js meshes.
 * NO map-specific conditionals — every element comes from the data.
 * The renderer understands object types (box, sphere, circle, line, plane)
 * and placement modes (instance, scatter, sequence, spline).
 */

import * as THREE from 'three'
import { logDebug } from '~/utils/debug'
import { generateHeightmap, generateHeightmapAsync } from '~/utils/heightmap'
import type { MapDefinition, MapObject, Placement } from '~/types/map-schema'
import type { MapCollisionBox, MapCollisionSphere, MapPhysicsData } from '~/types/physics'

export interface TerrainHandle {
  geometry: THREE.PlaneGeometry
  mesh: THREE.Mesh
  objects: THREE.Group
  boundaries: THREE.Group
  /** Authoritative collision/surface data for the physics worker. */
  collisionData: MapPhysicsData
  /** Resolves after an image-backed heightmap has replaced its flat bootstrap. */
  heightmapReady: Promise<void>
  getHeightAt(x: number, z: number): number
  getNormalAt(x: number, z: number): THREE.Vector3
  dispose(): void
}

export function useTerrainFromMap(map: MapDefinition): TerrainHandle {
  const segW = map.segments
  const segD = map.segments
  let heightData = map.terrain.heightmap
    ? new Float32Array((segW + 1) * (segD + 1))
    : generateHeightmap(map)

  // ── Terrain mesh ─────────────────────────────────────────────────────
  const geometry = new THREE.PlaneGeometry(map.size.width, map.size.depth, segW, segD)
  geometry.rotateX(-Math.PI / 2)

  const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute
  const applyHeightData = () => {
    for (let i = 0; i < posAttr.count; i++) {
      posAttr.setY(i, heightData[i] ?? 0)
    }
    posAttr.needsUpdate = true
    geometry.computeVertexNormals()
    geometry.computeBoundingSphere()
  }
  applyHeightData()

  const material = new THREE.MeshStandardMaterial({
    color: map.terrain.color,
    roughness: map.terrain.roughness,
    metalness: 0.0,
    flatShading: false,
  })

  const mesh = new THREE.Mesh(geometry, material)
  mesh.receiveShadow = true

  const collisionData: MapPhysicsData = {
    primitives: [],
    boundaries: [],
    roads: buildRoadSurfaceData(map),
    heightSamples: heightData,
    width: map.size.width,
    depth: map.size.depth,
    segments: map.segments,
  }

  const heightmapReady = map.terrain.heightmap
    ? generateHeightmapAsync(map).then((decoded) => {
      heightData = decoded
      collisionData.heightSamples = decoded
      applyHeightData()
      logDebug('terrain:heightmap-loaded', { map: map.id, samples: decoded.length })
    })
    : Promise.resolve()

  logDebug('terrain:initialized', {
    component: 'Terrain',
    map: map.id,
    width: map.size.width,
    depth: map.size.depth,
    segments: `${segW}x${segD}`,
    objectCount: map.objects.length,
  })

  function getHeightAt(wx: number, wz: number): number {
    const gx = (wx / map.size.width + 0.5) * segW
    const gz = (wz / map.size.depth + 0.5) * segD
    const ix = Math.floor(gx)
    const iz = Math.floor(gz)
    if (ix < 0 || ix >= segW || iz < 0 || iz >= segD) return 0

    const fx = gx - ix
    const fz = gz - iz
    const stride = segW + 1
    const h00 = heightData[iz * stride + ix] ?? 0
    const h10 = heightData[iz * stride + ix + 1] ?? h00
    const h01 = heightData[(iz + 1) * stride + ix] ?? h00
    const h11 = heightData[(iz + 1) * stride + ix + 1] ?? h01
    return (h00 + (h10 - h00) * fx) + ((h01 + (h11 - h01) * fx) - (h00 + (h10 - h00) * fx)) * fz
  }

  function getNormalAt(wx: number, wz: number): THREE.Vector3 {
    const e = 0.5
    return new THREE.Vector3(
      getHeightAt(wx - e, wz) - getHeightAt(wx + e, wz),
      2 * e,
      getHeightAt(wx, wz - e) - getHeightAt(wx, wz + e),
    ).normalize()
  }

  // ── Generic object renderer and authoritative collision data ────────────
  const objects = renderObjects(map, getHeightAt, collisionData)

  // ── Boundaries are data, not renderer-only walls ────────────────────────
  const boundaries = renderBoundaries(map, collisionData)

  function dispose() {
    geometry.dispose()
    material.dispose()
    disposeTree(objects)
    disposeTree(boundaries)
  }

  return { geometry, mesh, objects, boundaries, collisionData, heightmapReady, getHeightAt, getNormalAt, dispose }
}

/* ══════════════════════════════════════════════════════════════════════════
   GENERIC OBJECT RENDERER
   ══════════════════════════════════════════════════════════════════════════ */

function renderObjects(
  map: MapDefinition,
  getHeightAt: (x: number, z: number) => number,
  collisionData: MapPhysicsData,
): THREE.Group {
  const group = new THREE.Group()
  group.name = `kemudi-objects-${map.id}`

  for (const obj of map.objects) {
    const positions = resolvePlacement(obj.placement, map, getHeightAt)
    for (const pos of positions) {
      const mesh = createMesh(obj, pos, getHeightAt)
      if (mesh) group.add(mesh)
      const primitive = createCollisionPrimitive(obj, pos)
      if (primitive) collisionData.primitives.push(primitive)
    }
  }

  return group
}

/* ── Placement resolution ──────────────────────────────────────────────── */

interface ResolvedPosition {
  x: number
  y: number
  z: number
  rotation?: { x: number; y: number; z: number }
}

function resolvePlacement(
  placement: Placement,
  map: MapDefinition,
  getHeightAt: (x: number, z: number) => number,
): ResolvedPosition[] {
  switch (placement.mode) {
    case 'instance':
      return [{
        x: placement.position.x,
        y: placement.position.y ?? getHeightAt(placement.position.x, placement.position.z),
        z: placement.position.z,
        rotation: placement.rotation,
      }]

    case 'scatter':
      return resolveScatter(placement, map, getHeightAt)

    case 'sequence':
      return resolveSequence(placement, getHeightAt)

    case 'spline':
      return resolveSpline(placement, map, getHeightAt)
  }
}

function resolveScatter(
  p: NonNullable<Placement & { mode: 'scatter' }>,
  map: MapDefinition,
  getHeightAt: (x: number, z: number) => number,
): ResolvedPosition[] {
  const positions: ResolvedPosition[] = []
  const halfW = map.size.width * 0.5 * p.spread
  const halfD = map.size.depth * 0.5 * p.spread

  for (let i = 0; i < p.count; i++) {
    const rx = (pseudoRandom(p.seed + i * 3) - 0.5) * 2 * halfW
    const rz = (pseudoRandom(p.seed + i * 3 + 1) - 0.5) * 2 * halfD
    const scale = p.scale
      ? p.scale.min + pseudoRandom(p.seed + i * 3 + 2) * (p.scale.max - p.scale.min)
      : 1
    positions.push({
      x: rx,
      y: getHeightAt(rx, rz),
      z: rz,
      rotation: {
        x: pseudoRandom(p.seed + i * 7) * Math.PI,
        y: pseudoRandom(p.seed + i * 11) * Math.PI,
        z: 0,
      },
    })
    // Store scale in a side channel — createMesh reads it
    ;(positions[positions.length - 1] as ResolvedPosition & { _scale?: number })._scale = scale
  }
  return positions
}

function resolveSequence(
  p: NonNullable<Placement & { mode: 'sequence' }>,
  getHeightAt: (x: number, z: number) => number,
): ResolvedPosition[] {
  const positions: ResolvedPosition[] = []
  const terrainY = p.terrainOffset ?? 0

  for (let coord = p.start; coord <= p.end; coord += p.step) {
    const x = p.axis === 'x' ? coord : p.fixedCoord
    const z = p.axis === 'z' ? coord : p.fixedCoord
    positions.push({
      x,
      y: p.y ?? (getHeightAt(x, z) + terrainY),
      z,
    })
  }
  return positions
}

function resolveSpline(
  p: NonNullable<Placement & { mode: 'spline' }>,
  map: MapDefinition,
  getHeightAt: (x: number, z: number) => number,
): ResolvedPosition[] {
  const road = map.roads.find(r => r.name === p.roadName)
  if (!road || road.points.length < 2) return []

  const positions: ResolvedPosition[] = []
  const terrainY = p.terrainOffset ?? 0

  // Walk the spline at fixed intervals
  let accumulated = 0
  for (let i = 0; i < road.points.length - 1; i++) {
    const a = road.points[i]!
    const b = road.points[i + 1]!
    const dx = b.x - a.x
    const dz = b.z - a.z
    const segLen = Math.sqrt(dx * dx + dz * dz)
    if (segLen < 0.001) continue

    // Perpendicular for offset
    const perpX = -dz / segLen
    const perpZ = dx / segLen

    let localDist = 0
    while (localDist < segLen) {
      const t = localDist / segLen
      const px = a.x + dx * t + perpX * p.offset
      const pz = a.z + dz * t + perpZ * p.offset
      positions.push({
        x: px,
        y: getHeightAt(px, pz) + terrainY,
        z: pz,
      })
      localDist += p.spacing
      accumulated += p.spacing
    }
  }

  return positions
}

/* ── Mesh creation ─────────────────────────────────────────────────────── */

function createMesh(
  obj: MapObject,
  pos: ResolvedPosition & { _scale?: number },
  getHeightAt: (x: number, z: number) => number,
): THREE.Object3D | null {
  const vis = obj.visual
  const mat = new THREE.MeshStandardMaterial({
    color: vis.color ?? 0x888888,
    roughness: vis.roughness ?? 0.7,
    metalness: vis.metalness ?? 0.0,
    flatShading: vis.flatShading ?? false,
    transparent: vis.opacity !== undefined && vis.opacity < 1,
    opacity: vis.opacity ?? 1.0,
  })

  switch (obj.type) {
    case 'box': {
      const s = vis.size ?? [1, 1, 1]
      const geo = new THREE.BoxGeometry(s[0], s[1], s[2])
      const mesh = new THREE.Mesh(geo, mat)
      mesh.position.set(pos.x, pos.y + s[1] / 2, pos.z)
      if (pos.rotation) mesh.rotation.set(pos.rotation.x, pos.rotation.y, pos.rotation.z)
      mesh.castShadow = vis.castShadow ?? false
      mesh.receiveShadow = vis.receiveShadow ?? false
      mesh.name = obj.name ?? 'box'
      return mesh
    }

    case 'sphere': {
      const scale = pos._scale ?? 1
      const r = vis.radius ?? scale
      const detail = scale > 2 ? 2 : 1
      const geo = new THREE.DodecahedronGeometry(r, detail)

      // Deform vertices for natural look using scatter seed
      const posAttr = geo.getAttribute('position') as THREE.BufferAttribute
      const seed = obj.placement.mode === 'scatter' ? obj.placement.seed : 0
      for (let v = 0; v < posAttr.count; v++) {
        const n = 0.7 + pseudoRandom(seed + v) * 0.6
        posAttr.setX(v, posAttr.getX(v) * n)
        posAttr.setY(v, posAttr.getY(v) * (0.5 + pseudoRandom(seed + v + 50) * 0.4))
        posAttr.setZ(v, posAttr.getZ(v) * n)
      }
      geo.computeVertexNormals()

      const mesh = new THREE.Mesh(geo, mat)
      mesh.position.set(pos.x, pos.y + r * 0.3, pos.z)
      if (pos.rotation) mesh.rotation.set(pos.rotation.x, pos.rotation.y, pos.rotation.z)
      mesh.castShadow = vis.castShadow ?? false
      mesh.name = obj.name ?? 'sphere'
      return mesh
    }

    case 'circle': {
      const r = pos._scale ?? vis.circleRadius ?? 5
      const geo = new THREE.CircleGeometry(r, 16)
      geo.rotateX(-Math.PI / 2)

      // Sample terrain height for each vertex
      const posAttr = geo.getAttribute('position') as THREE.BufferAttribute
      for (let v = 0; v < posAttr.count; v++) {
        posAttr.setY(v, getHeightAt(posAttr.getX(v) + pos.x, posAttr.getZ(v) + pos.z) + 0.02)
      }
      posAttr.needsUpdate = true
      geo.computeVertexNormals()

      const mesh = new THREE.Mesh(geo, mat)
      mesh.position.set(pos.x, 0, pos.z)
      mesh.receiveShadow = vis.receiveShadow ?? false
      mesh.name = obj.name ?? 'circle'
      return mesh
    }

    case 'line': {
      if (!vis.linePoints || vis.linePoints.length < 6) return null

      // For sequence/spline placement, offset the line points to the instance position
      const isLocal = obj.placement.mode === 'instance'
      const pts = new Float32Array(vis.linePoints.length)
      for (let i = 0; i < vis.linePoints.length; i += 3) {
        pts[i] = vis.linePoints[i]! + (isLocal ? pos.x : 0)
        pts[i + 1] = vis.linePoints[i + 1]! + (isLocal ? pos.y : 0)
        pts[i + 2] = vis.linePoints[i + 2]! + (isLocal ? pos.z : 0)
      }

      // For sequence placement, translate each instance to its position
      if (!isLocal) {
        for (let i = 0; i < pts.length; i += 3) {
          const baseX = (pts[i] ?? 0) + pos.x
          const baseZ = (pts[i + 2] ?? 0) + pos.z
          pts[i] = baseX
          pts[i + 1] = getHeightAt(baseX, baseZ) + (vis.linePoints?.[i + 1] ?? 0)
          pts[i + 2] = baseZ
        }
      }

      const geo = new THREE.BufferGeometry()
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3))

      const lineMat = new THREE.LineBasicMaterial({
        color: vis.color ?? 0xffffff,
        transparent: vis.opacity !== undefined && vis.opacity < 1,
        opacity: vis.opacity ?? 1.0,
      })
      const lines = new THREE.LineSegments(geo, lineMat)
      lines.name = obj.name ?? 'line'
      return lines
    }

    case 'plane': {
      const s = vis.size ?? [10, 1, 10]
      const geo = new THREE.PlaneGeometry(s[0], s[2], 1, Math.max(4, Math.round(s[2] / 4)))
      geo.rotateX(-Math.PI / 2)

      // Sample terrain for each vertex
      const posAttr = geo.getAttribute('position') as THREE.BufferAttribute
      for (let v = 0; v < posAttr.count; v++) {
        const vx = posAttr.getX(v) + pos.x
        const vz = posAttr.getZ(v) + pos.z
        posAttr.setY(v, getHeightAt(vx, vz) + pos.y)
      }
      posAttr.needsUpdate = true
      geo.computeVertexNormals()

      const mesh = new THREE.Mesh(geo, mat)
      mesh.receiveShadow = vis.receiveShadow ?? false
      mesh.name = obj.name ?? 'plane'
      return mesh
    }

    default:
      return null
  }
}

function createCollisionPrimitive(obj: MapObject, pos: ResolvedPosition & { _scale?: number }): MapCollisionBox | MapCollisionSphere | null {
  const shape = obj.collision ?? (obj.type === 'box'
    ? { type: 'box' as const, size: obj.visual.size ?? [1, 1, 1] as [number, number, number] }
    : obj.type === 'sphere'
      ? { type: 'sphere' as const, radius: obj.visual.radius ?? pos._scale ?? 1 }
      : undefined)
  if (!shape) return null
  const offset = shape.offset ?? {}
  const center = { x: pos.x + (offset.x ?? 0), y: pos.y + (offset.y ?? 0), z: pos.z + (offset.z ?? 0) }
  const restitution = Math.min(1, Math.max(0, shape.restitution ?? 0.1))
  const friction = Math.min(1, Math.max(0, shape.friction ?? 0.8))
  if (shape.type === 'box') {
    const size = shape.size
    return { kind: 'box', center: { x: center.x, y: center.y + size[1] / 2, z: center.z }, halfExtents: { x: size[0] / 2, y: size[1] / 2, z: size[2] / 2 }, restitution, friction }
  }
  const radius = Math.max(0.01, shape.radius)
  return { kind: 'sphere', center: { x: center.x, y: center.y + radius, z: center.z }, radius, restitution, friction }
}

function buildRoadSurfaceData(map: MapDefinition): MapPhysicsData['roads'] {
  return map.roads.slice(0, 1024).map((road) => ({
    name: road.name,
    width: Math.max(0.01, road.width),
    surfaceId: Math.max(0, Math.min(255, Math.trunc(road.surfaceId))),
    friction: road.surface?.friction ?? Math.min(1.5, Math.max(0, map.terrain.groundFriction)),
    roughness: road.surface?.roughness ?? map.terrain.roughness,
    moisture: road.surface?.moisture ?? 0,
    compactness: road.surface?.compactness ?? 1,
    points: road.points.slice(0, 4096).map(point => ({ x: point.x, z: point.z })),
  }))
}

/* ── Boundaries ────────────────────────────────────────────────────────── */

function renderBoundaries(map: MapDefinition, collisionData: MapPhysicsData): THREE.Group {
  const group = new THREE.Group()
  group.name = `kemudi-boundaries-${map.id}`
  const definitions = map.boundaries ?? []
  for (const boundary of definitions.slice(0, 64)) {
    const size = boundary.size
    const position = boundary.position
    const center = { x: position.x, y: position.y ?? 0, z: position.z }
    const primitive: MapCollisionBox = {
      kind: 'box',
      center,
      halfExtents: { x: size[0] / 2, y: size[1] / 2, z: size[2] / 2 },
      restitution: Math.min(1, Math.max(0, boundary.restitution ?? 0.1)),
      friction: Math.min(1, Math.max(0, boundary.friction ?? 0.8)),
    }
    collisionData.boundaries.push(primitive)
    // Keep the render tree generic and invisible; collisionData is authoritative.
    const geometry = new THREE.BoxGeometry(size[0], size[1], size[2])
    const material = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false })
    const mesh = new THREE.Mesh(geometry, material)
    mesh.position.set(center.x, center.y, center.z)
    if (boundary.rotation) mesh.rotation.set(boundary.rotation.x ?? 0, boundary.rotation.y ?? 0, boundary.rotation.z ?? 0)
    mesh.visible = false
    mesh.name = boundary.name ?? 'boundary'
    group.add(mesh)
  }
  return group
}

/* ── Utilities ─────────────────────────────────────────────────────────── */

function pseudoRandom(seed: number): number {
  const x = Math.sin(seed * 127.1 + 311.7) * 43758.5453
  return x - Math.floor(x)
}

function disposeTree(root: THREE.Object3D): void {
  const geometries = new Set<THREE.BufferGeometry>()
  const materials = new Set<THREE.Material>()
  root.traverse((obj) => {
    if (!(obj instanceof THREE.Mesh || obj instanceof THREE.LineSegments)) return
    if (obj.geometry) geometries.add(obj.geometry)
    const m = obj.material
    if (Array.isArray(m)) { for (const e of m) materials.add(e) }
    else if (m) materials.add(m)
  })
  for (const g of geometries) g.dispose()
  for (const m of materials) m.dispose()
}
