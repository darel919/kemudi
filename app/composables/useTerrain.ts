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

  function dispose() {
    geometry.dispose()
    material.dispose()
  }

  return { geometry, mesh, getHeightAt, getNormalAt, dispose }
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
