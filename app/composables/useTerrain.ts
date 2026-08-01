import * as THREE from 'three'
import { logDebug } from '~/utils/debug'

export interface TerrainConfig {
  width: number
  depth: number
  segmentsW: number
  segmentsD: number
  heightScale: number
  profile?: 'flat' | 'bumpy' | 'offroad'
  color?: number
  roughness?: number
}

const DEFAULT_TERRAIN: TerrainConfig = {
  width: 400,
  depth: 400,
  segmentsW: 128,
  segmentsD: 128,
  heightScale: 20,
  profile: 'bumpy',
  color: 0x556644,
  roughness: 0.9,
}

export interface TerrainHandle {
  geometry: THREE.PlaneGeometry
  mesh: THREE.Mesh
  /** Visible route and range markers that make the drivable surface legible. */
  decorations: THREE.Group
  /** Get height at world (x, z) via bilinear sampling */
  getHeightAt(x: number, z: number): number
  /** Get surface normal at world (x, z) */
  getNormalAt(x: number, z: number): THREE.Vector3
  dispose(): void
}

export function useTerrain(config: Partial<TerrainConfig> = {}): TerrainHandle {
  const cfg = { ...DEFAULT_TERRAIN, ...config }

  const geometry = new THREE.PlaneGeometry(
    cfg.width, cfg.depth,
    cfg.segmentsW, cfg.segmentsD,
  )
  // Rotate to XZ plane
  geometry.rotateX(-Math.PI / 2)

  const heightData = new Float32Array((cfg.segmentsW + 1) * (cfg.segmentsD + 1))

  // Simple procedural height: flat with gentle rolling hills
  const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute
  for (let i = 0; i < posAttr.count; i++) {
    const x = posAttr.getX(i)
    const z = posAttr.getZ(i)
    const h = terrainHeight(x, z, cfg.heightScale, cfg.profile ?? 'bumpy')
    posAttr.setY(i, h)
    heightData[i] = h
  }
  posAttr.needsUpdate = true
  geometry.computeVertexNormals()
  geometry.computeBoundingSphere()

  const material = new THREE.MeshStandardMaterial({
    color: cfg.color ?? 0x556644,
    roughness: cfg.roughness ?? 0.9,
    metalness: 0.0,
    flatShading: false,
  })

  const mesh = new THREE.Mesh(geometry, material)
  mesh.receiveShadow = true

  logDebug('terrain:initialized', {
    component: 'Terrain',
    width: cfg.width,
    depth: cfg.depth,
    segments: `${cfg.segmentsW}x${cfg.segmentsD}`,
  })

  function getHeightAt(wx: number, wz: number): number {
    // Map world coords to grid
    const gx = (wx / cfg.width + 0.5) * cfg.segmentsW
    const gz = (wz / cfg.depth + 0.5) * cfg.segmentsD

    const ix = Math.floor(gx)
    const iz = Math.floor(gz)
    if (ix < 0 || ix >= cfg.segmentsW || iz < 0 || iz >= cfg.segmentsD) return 0

    const fx = gx - ix
    const fz = gz - iz

    const stride = cfg.segmentsW + 1
    const h00 = heightData[iz * stride + ix] ?? 0
    const h10 = heightData[iz * stride + ix + 1] ?? h00
    const h01 = heightData[(iz + 1) * stride + ix] ?? h00
    const h11 = heightData[(iz + 1) * stride + ix + 1] ?? h01

    // Bilinear interpolation
    const h0 = h00 + (h10 - h00) * fx
    const h1 = h01 + (h11 - h01) * fx
    return h0 + (h1 - h0) * fz
  }

  function getNormalAt(wx: number, wz: number): THREE.Vector3 {
    const e = 0.5
    const hL = getHeightAt(wx - e, wz)
    const hR = getHeightAt(wx + e, wz)
    const hD = getHeightAt(wx, wz - e)
    const hU = getHeightAt(wx, wz + e)
    return new THREE.Vector3(hL - hR, 2 * e, hD - hU).normalize()
  }

  const decorations = createRouteDecorations(cfg, getHeightAt)

  function dispose() {
    geometry.dispose()
    material.dispose()
    disposeObjectTree(decorations)
  }

  return { geometry, mesh, decorations, getHeightAt, getNormalAt, dispose }
}

function createRouteDecorations(
  cfg: TerrainConfig,
  getHeightAt: (x: number, z: number) => number,
): THREE.Group {
  const group = new THREE.Group()
  group.name = 'kemudi-route-decorations'

  const profile = cfg.profile ?? 'flat'
  const routeLength = Math.max(40, Math.min(cfg.depth - 8, 280))
  const routeWidth = profile === 'offroad' ? 7.5 : 9
  const routeGeometry = new THREE.PlaneGeometry(
    routeWidth,
    routeLength,
    1,
    Math.max(24, Math.round(routeLength / 4)),
  )
  routeGeometry.rotateX(-Math.PI / 2)
  const routePositions = routeGeometry.getAttribute('position') as THREE.BufferAttribute
  for (let i = 0; i < routePositions.count; i++) {
    routePositions.setY(
      i,
      getHeightAt(routePositions.getX(i), routePositions.getZ(i)) + 0.018,
    )
  }
  routePositions.needsUpdate = true
  routeGeometry.computeVertexNormals()

  const routeMaterial = new THREE.MeshStandardMaterial({
    color: profile === 'offroad' ? 0x5b4631 : 0x1b2329,
    roughness: profile === 'offroad' ? 1 : 0.92,
    metalness: 0.02,
  })
  const route = new THREE.Mesh(routeGeometry, routeMaterial)
  route.name = 'route-surface'
  route.receiveShadow = true
  group.add(route)

  const lineMaterial = new THREE.LineBasicMaterial({
    color: profile === 'offroad' ? 0xc19b62 : 0xe9d99a,
    transparent: true,
    opacity: profile === 'offroad' ? 0.5 : 0.9,
  })
  const linePoints: number[] = []
  const routeStart = -routeLength / 2 + 5
  const routeEnd = routeLength / 2 - 5
  const edgeX = routeWidth / 2 - 0.35
  const addSegment = (x: number, z0: number, z1: number) => {
    linePoints.push(
      x, getHeightAt(x, z0) + 0.035, z0,
      x, getHeightAt(x, z1) + 0.035, z1,
    )
  }
  if (profile === 'offroad') {
    for (let z = routeStart; z < routeEnd; z += 10) {
      addSegment(-1.35, z, Math.min(z + 5, routeEnd))
      addSegment(1.35, z, Math.min(z + 5, routeEnd))
    }
  } else {
    for (let z = routeStart; z < routeEnd; z += 12) {
      addSegment(0, z, Math.min(z + 6, routeEnd))
    }
    addSegment(-edgeX, routeStart, routeEnd)
    addSegment(edgeX, routeStart, routeEnd)
  }
  const lineGeometry = new THREE.BufferGeometry()
  lineGeometry.setAttribute('position', new THREE.Float32BufferAttribute(linePoints, 3))
  const routeLines = new THREE.LineSegments(lineGeometry, lineMaterial)
  routeLines.name = 'route-markings'
  group.add(routeLines)

  const startPoints: number[] = []
  for (let row = -2; row <= 2; row++) {
    const z = 4 + row * 0.7
    startPoints.push(
      -routeWidth / 2 + 0.45, getHeightAt(-routeWidth / 2 + 0.45, z) + 0.042, z,
      routeWidth / 2 - 0.45, getHeightAt(routeWidth / 2 - 0.45, z) + 0.042, z,
    )
  }
  const startGeometry = new THREE.BufferGeometry()
  startGeometry.setAttribute('position', new THREE.Float32BufferAttribute(startPoints, 3))
  const startLine = new THREE.LineSegments(
    startGeometry,
    new THREE.LineBasicMaterial({ color: 0xf5f7ec, transparent: true, opacity: 0.8 }),
  )
  startLine.name = 'start-grid'
  group.add(startLine)

  const postGeometry = new THREE.BoxGeometry(0.1, 0.42, 0.1)
  const postMaterial = new THREE.MeshStandardMaterial({
    color: profile === 'offroad' ? 0xd49b55 : 0x8ee6cc,
    roughness: 0.65,
    metalness: 0.15,
  })
  const postOffset = routeWidth / 2 + 1.4
  for (let z = routeStart + 8; z <= routeEnd; z += 20) {
    for (const x of [-postOffset, postOffset]) {
      const post = new THREE.Mesh(postGeometry, postMaterial)
      post.position.set(x, getHeightAt(x, z) + 0.21, z)
      post.castShadow = true
      post.name = 'route-marker'
      group.add(post)
    }
  }

  return group
}

function disposeObjectTree(root: THREE.Object3D): void {
  const geometries = new Set<THREE.BufferGeometry>()
  const materials = new Set<THREE.Material>()
  root.traverse((object) => {
    if (!(object instanceof THREE.Mesh || object instanceof THREE.LineSegments)) return
    if (object.geometry) geometries.add(object.geometry)
    const material = object.material
    if (Array.isArray(material)) {
      for (const entry of material) materials.add(entry)
    } else if (material) {
      materials.add(material)
    }
  })
  for (const geometry of geometries) geometry.dispose()
  for (const material of materials) material.dispose()
}

function terrainHeight(x: number, z: number, scale: number, profile: TerrainConfig['profile']): number {
  if (profile === 'flat' || scale === 0) return 0

  if (profile === 'offroad') {
    const broad = Math.sin(x * 0.035) * 0.55 + Math.cos(z * 0.027) * 0.4
    const ridges = Math.sin((x - z) * 0.09) * 0.22 + Math.cos((x + z) * 0.065) * 0.16
    return (broad + ridges) * scale * 0.12
  }

  // Bumpy road: shallow, repeated undulations instead of a smooth hill.
  return (
    Math.sin(x * 0.12) * 0.28 +
    Math.sin(z * 0.17) * 0.18 +
    Math.sin((x + z) * 0.045) * 0.14
  ) * scale * 0.08
}
