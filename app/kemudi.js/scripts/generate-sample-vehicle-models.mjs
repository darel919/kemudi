import { mkdir, writeFile } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'

const outputDirectory = resolve('public/models')

class MeshBuilder {
  constructor() {
    this.positions = []
    this.indices = []
  }

  addFace(points, center) {
    const start = this.positions.length / 3
    for (const point of points) this.positions.push(...point)

    const a = points[0]
    const b = points[1]
    const c = points[2]
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
    const ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]]
    const normal = [
      ab[1] * ac[2] - ab[2] * ac[1],
      ab[2] * ac[0] - ab[0] * ac[2],
      ab[0] * ac[1] - ab[1] * ac[0],
    ]
    const faceCenter = points.reduce((sum, point) => [
      sum[0] + point[0] / points.length,
      sum[1] + point[1] / points.length,
      sum[2] + point[2] / points.length,
    ], [0, 0, 0])
    const outward = [faceCenter[0] - center[0], faceCenter[1] - center[1], faceCenter[2] - center[2]]
    const outwardDot = normal[0] * outward[0] + normal[1] * outward[1] + normal[2] * outward[2]

    if (outwardDot >= 0) {
      this.indices.push(start, start + 1, start + 2, start, start + 2, start + 3)
    } else {
      this.indices.push(start, start + 2, start + 1, start, start + 3, start + 2)
    }
  }

  addBox({ x0, x1, y0, y1, z0, z1 }) {
    const center = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]
    this.addFace([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0]], center)
    this.addFace([[x1, y0, z1], [x0, y0, z1], [x0, y1, z1], [x1, y1, z1]], center)
    this.addFace([[x0, y0, z1], [x0, y0, z0], [x0, y1, z0], [x0, y1, z1]], center)
    this.addFace([[x1, y0, z0], [x1, y0, z1], [x1, y1, z1], [x1, y1, z0]], center)
    this.addFace([[x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1]], center)
    this.addFace([[x0, y0, z1], [x1, y0, z1], [x1, y0, z0], [x0, y0, z0]], center)
  }

  addFrustum({ bottomX, bottomZ, topX, topZ, y0, y1, zCenter = 0 }) {
    const center = [0, (y0 + y1) / 2, zCenter]
    this.addFace([
      [-bottomX, y0, zCenter - bottomZ],
      [bottomX, y0, zCenter - bottomZ],
      [bottomX, y1, zCenter - topZ],
      [-bottomX, y1, zCenter - topZ],
    ], center)
    this.addFace([
      [bottomX, y0, zCenter + bottomZ],
      [-bottomX, y0, zCenter + bottomZ],
      [-bottomX, y1, zCenter + topZ],
      [bottomX, y1, zCenter + topZ],
    ], center)
    this.addFace([
      [-bottomX, y0, zCenter + bottomZ],
      [-bottomX, y0, zCenter - bottomZ],
      [-bottomX, y1, zCenter - topZ],
      [-topX, y1, zCenter + topZ],
    ], center)
    this.addFace([
      [bottomX, y0, zCenter - bottomZ],
      [bottomX, y0, zCenter + bottomZ],
      [topX, y1, zCenter + topZ],
      [topX, y1, zCenter - topZ],
    ], center)
    this.addFace([
      [-topX, y1, zCenter - topZ],
      [topX, y1, zCenter - topZ],
      [topX, y1, zCenter + topZ],
      [-topX, y1, zCenter + topZ],
    ], center)
    this.addFace([
      [-bottomX, y0, zCenter + bottomZ],
      [bottomX, y0, zCenter + bottomZ],
      [bottomX, y0, zCenter - bottomZ],
      [-bottomX, y0, zCenter - bottomZ],
    ], center)
  }

  addProfileExtrusion(profile, z0, z1) {
    const center = [0, profile.reduce((sum, point) => sum + point[1], 0) / profile.length, (z0 + z1) / 2]
    const front = profile.map(([x, y]) => [x, y, z0])
    const back = profile.map(([x, y]) => [x, y, z1])
    this.addFace([...front].reverse(), center)
    this.addFace(back, center)
    for (let i = 0; i < profile.length; i++) {
      const next = (i + 1) % profile.length
      this.addFace([front[i], front[next], back[next], back[i]], center)
    }
  }

  build() {
    const normals = new Float32Array(this.positions.length)
    for (let i = 0; i < this.indices.length; i += 3) {
      const ia = this.indices[i] * 3
      const ib = this.indices[i + 1] * 3
      const ic = this.indices[i + 2] * 3
      const ax = this.positions[ia]
      const ay = this.positions[ia + 1]
      const az = this.positions[ia + 2]
      const abx = this.positions[ib] - ax
      const aby = this.positions[ib + 1] - ay
      const abz = this.positions[ib + 2] - az
      const acx = this.positions[ic] - ax
      const acy = this.positions[ic + 1] - ay
      const acz = this.positions[ic + 2] - az
      const nx = aby * acz - abz * acy
      const ny = abz * acx - abx * acz
      const nz = abx * acy - aby * acx
      for (const index of [ia, ib, ic]) {
        normals[index] += nx
        normals[index + 1] += ny
        normals[index + 2] += nz
      }
    }
    for (let i = 0; i < normals.length; i += 3) {
      const length = Math.hypot(normals[i], normals[i + 1], normals[i + 2]) || 1
      normals[i] /= length
      normals[i + 1] /= length
      normals[i + 2] /= length
    }
    return {
      positions: new Float32Array(this.positions),
      normals,
      indices: new Uint16Array(this.indices),
    }
  }
}

function makeModel(name, build) {
  const builder = new MeshBuilder()
  build(builder)
  return { name, ...builder.build() }
}

const models = [
  makeModel('basic_car', (mesh) => {
    mesh.addProfileExtrusion([
      [-0.63, 0.05], [0.63, 0.05], [0.70, 0.12], [0.70, 0.31],
      [0.62, 0.38], [-0.62, 0.38], [-0.70, 0.31], [-0.70, 0.12],
    ], -0.52, 0.52)
    mesh.addFrustum({ bottomX: 0.58, bottomZ: 0.34, topX: 0.43, topZ: 0.24, y0: 0.30, y1: 0.57, zCenter: 0.02 })
    mesh.addBox({ x0: -0.64, x1: 0.64, y0: 0.22, y1: 0.31, z0: -0.56, z1: -0.51 })
    mesh.addBox({ x0: -0.64, x1: 0.64, y0: 0.22, y1: 0.31, z0: 0.51, z1: 0.56 })
  }),
  makeModel('basic_truck', (mesh) => {
    mesh.addProfileExtrusion([
      [-0.72, 0.05], [0.72, 0.05], [0.79, 0.13], [0.79, 0.46],
      [0.71, 0.53], [-0.71, 0.53], [-0.79, 0.46], [-0.79, 0.13],
    ], -0.72, 0.72)
    mesh.addFrustum({ bottomX: 0.69, bottomZ: 0.38, topX: 0.61, topZ: 0.32, y0: 0.42, y1: 0.94, zCenter: -0.39 })
    mesh.addBox({ x0: -0.72, x1: 0.72, y0: 0.39, y1: 0.66, z0: 0.02, z1: 0.67 })
    mesh.addBox({ x0: -0.77, x1: 0.77, y0: 0.53, y1: 0.66, z0: 0.61, z1: 0.71 })
    mesh.addBox({ x0: -0.72, x1: -0.64, y0: 0.54, y1: 0.78, z0: 0.03, z1: 0.67 })
    mesh.addBox({ x0: 0.64, x1: 0.72, y0: 0.54, y1: 0.78, z0: 0.03, z1: 0.67 })
  }),
  makeModel('basic_atv', (mesh) => {
    mesh.addFrustum({ bottomX: 0.40, bottomZ: 0.47, topX: 0.31, topZ: 0.30, y0: 0.08, y1: 0.43, zCenter: 0.02 })
    mesh.addFrustum({ bottomX: 0.28, bottomZ: 0.25, topX: 0.23, topZ: 0.17, y0: 0.38, y1: 0.58, zCenter: 0.12 })
    mesh.addBox({ x0: -0.43, x1: -0.34, y0: 0.16, y1: 0.28, z0: -0.48, z1: 0.48 })
    mesh.addBox({ x0: 0.34, x1: 0.43, y0: 0.16, y1: 0.28, z0: -0.48, z1: 0.48 })
    mesh.addBox({ x0: -0.04, x1: 0.04, y0: 0.47, y1: 0.63, z0: -0.42, z1: -0.34 })
    mesh.addBox({ x0: -0.30, x1: 0.30, y0: 0.54, y1: 0.60, z0: -0.44, z1: -0.37 })
  }),
  makeModel('premium_sportscar', (mesh) => {
    mesh.addProfileExtrusion([
      [-0.64, 0.04], [0.64, 0.04], [0.74, 0.11], [0.72, 0.28],
      [0.58, 0.36], [-0.58, 0.36], [-0.72, 0.28], [-0.74, 0.11],
    ], -0.62, 0.62)
    mesh.addFrustum({ bottomX: 0.56, bottomZ: 0.34, topX: 0.40, topZ: 0.21, y0: 0.28, y1: 0.66, zCenter: 0.04 })
    mesh.addBox({ x0: -0.68, x1: 0.68, y0: 0.26, y1: 0.33, z0: -0.65, z1: -0.56 })
    mesh.addBox({ x0: -0.68, x1: 0.68, y0: 0.25, y1: 0.32, z0: 0.56, z1: 0.65 })
    mesh.addBox({ x0: -0.58, x1: 0.58, y0: 0.34, y1: 0.40, z0: -0.46, z1: -0.36 })
  }),
]

function align4(value) {
  return (value + 3) & ~3
}

function createGlb(model) {
  const positionBytes = new Uint8Array(model.positions.buffer)
  const normalBytes = new Uint8Array(model.normals.buffer)
  const indexBytes = new Uint8Array(model.indices.buffer)
  const positionOffset = 0
  const normalOffset = align4(positionOffset + positionBytes.byteLength)
  const indexOffset = align4(normalOffset + normalBytes.byteLength)
  const binary = new Uint8Array(align4(indexOffset + indexBytes.byteLength))
  binary.set(positionBytes, positionOffset)
  binary.set(normalBytes, normalOffset)
  binary.set(indexBytes, indexOffset)

  const json = JSON.stringify({
    asset: { version: '2.0', generator: 'Kemudi.js sample vehicle model generator' },
    scene: 0,
    scenes: [{ nodes: [0] }],
    nodes: [{ name: `${model.name} body`, mesh: 0 }],
    meshes: [{
      name: `${model.name} body mesh`,
      primitives: [{
        attributes: { POSITION: 0, NORMAL: 1 },
        indices: 2,
        material: 0,
      }],
    }],
    materials: [{
      name: 'sample body material',
      pbrMetallicRoughness: { baseColorFactor: [0.22, 0.48, 0.72, 1], metallicFactor: 0.35, roughnessFactor: 0.42 },
    }],
    accessors: [
      {
        bufferView: 0,
        componentType: 5126,
        count: model.positions.length / 3,
        type: 'VEC3',
        min: [Math.min(...model.positions.filter((_, i) => i % 3 === 0)), Math.min(...model.positions.filter((_, i) => i % 3 === 1)), Math.min(...model.positions.filter((_, i) => i % 3 === 2))],
        max: [Math.max(...model.positions.filter((_, i) => i % 3 === 0)), Math.max(...model.positions.filter((_, i) => i % 3 === 1)), Math.max(...model.positions.filter((_, i) => i % 3 === 2))],
      },
      { bufferView: 1, componentType: 5126, count: model.normals.length / 3, type: 'VEC3' },
      { bufferView: 2, componentType: 5123, count: model.indices.length, type: 'SCALAR', min: [0], max: [model.positions.length / 3 - 1] },
    ],
    bufferViews: [
      { buffer: 0, byteOffset: positionOffset, byteLength: positionBytes.byteLength, target: 34962 },
      { buffer: 0, byteOffset: normalOffset, byteLength: normalBytes.byteLength, target: 34962 },
      { buffer: 0, byteOffset: indexOffset, byteLength: indexBytes.byteLength, target: 34963 },
    ],
    buffers: [{ byteLength: binary.byteLength }],
  })

  const jsonBytes = new TextEncoder().encode(json)
  const jsonChunkLength = align4(jsonBytes.byteLength)
  const totalLength = 12 + 8 + jsonChunkLength + 8 + binary.byteLength
  const glb = new Uint8Array(totalLength)
  const view = new DataView(glb.buffer)
  view.setUint32(0, 0x46546c67, true)
  view.setUint32(4, 2, true)
  view.setUint32(8, totalLength, true)
  view.setUint32(12, jsonChunkLength, true)
  view.setUint32(16, 0x4e4f534a, true)
  glb.set(jsonBytes, 20)
  glb.fill(0x20, 20 + jsonBytes.byteLength, 20 + jsonChunkLength)
  const binaryHeaderOffset = 20 + jsonChunkLength
  view.setUint32(binaryHeaderOffset, binary.byteLength, true)
  view.setUint32(binaryHeaderOffset + 4, 0x004e4942, true)
  glb.set(binary, binaryHeaderOffset + 8)
  return glb
}

await mkdir(outputDirectory, { recursive: true })
for (const model of models) {
  const outputPath = resolve(outputDirectory, `${model.name}.body.glb`)
  await writeFile(outputPath, createGlb(model))
  console.log(`${outputPath}: ${model.positions.length / 3} vertices, ${model.indices.length / 3} triangles`)
}
