/**
 * Heightmap generation and loading utilities.
 *
 * Supports two modes:
 *  1. Procedural — generates height data from wave definitions in map.json
 *  2. Image-based — loads a grayscale PNG heightmap (future)
 *
 * Returns a Float32Array of elevation values (row-major, matching vertex grid).
 */

import type { MapDefinition, ProceduralConfig, WaveTerm } from '~/types/map-schema'

/**
 * Generate or load height data for a map definition.
 * Returns a Float32Array of length (segmentsW+1) * (segmentsD+1).
 */
export function generateHeightmap(map: MapDefinition): Float32Array {
  if (map.terrain.heightmap) {
    throw new Error(`Image heightmap requires generateHeightmapAsync(): ${map.terrain.heightmap}`)
  }

  if (map.terrain.procedural) {
    return generateProceduralHeightmap(map)
  }

  // Flat terrain (heightScale = 0, no procedural config)
  return new Float32Array((map.segments + 1) * (map.segments + 1))
}

export interface DecodedHeightmap {
  pixels: Uint8ClampedArray
  width: number
  height: number
}

export interface HeightmapLoadOptions {
  /** World elevation range is [-heightScale, heightScale]. */
  heightScale: number
}

/** Convert browser image grayscale bytes into bounded world elevations. */
export function normalizeHeightmapPixels(
  pixels: Uint8ClampedArray,
  width: number,
  heightScale: number,
): Float32Array {
  if (!Number.isInteger(width) || width <= 0 || pixels.length === 0 || pixels.length % width !== 0) {
    throw new Error('heightmap pixel buffer must contain complete rows')
  }
  if (!Number.isFinite(heightScale) || heightScale < 0 || heightScale > 100_000) {
    throw new Error('heightmap scale must be finite and bounded')
  }
  const output = new Float32Array(pixels.length)
  for (let i = 0; i < pixels.length; i++) {
    const sample = pixels[i]!
    // Treat the two representable midpoint codes as the authored zero
    // datum; this avoids a visible one-code bias on 8-bit heightmaps.
    output[i] = (sample === 127 || sample === 128 ? 0 : (sample / 255 * 2 - 1)) * heightScale
  }
  return output
}

/** Decode a PNG/JPEG image through browser APIs; never fakes them in sync code. */
export async function loadImageHeightmap(
  source: string | URL | Blob,
  options: HeightmapLoadOptions,
): Promise<DecodedHeightmap> {
  if (!Number.isFinite(options.heightScale) || options.heightScale < 0 || options.heightScale > 100_000) {
    throw new Error('heightmap scale must be finite and bounded')
  }
  const blob = typeof source === 'string' || source instanceof URL
    ? await fetch(String(source)).then(response => {
      if (!response.ok) throw new Error(`Failed to load heightmap: ${response.status}`)
      return response.blob()
    })
    : source
  if (typeof createImageBitmap !== 'function') {
    throw new Error('Image heightmaps require browser image decoding support')
  }
  const bitmap = await createImageBitmap(blob)
  try {
    const canvas = typeof OffscreenCanvas !== 'undefined'
      ? new OffscreenCanvas(bitmap.width, bitmap.height)
      : createDocumentCanvas(bitmap.width, bitmap.height)
    const context = canvas.getContext('2d') as CanvasRenderingContext2D | OffscreenCanvasRenderingContext2D | null
    if (!context) throw new Error('Unable to create a 2D canvas for heightmap decoding')
    context.drawImage(bitmap, 0, 0)
    const image = context.getImageData(0, 0, bitmap.width, bitmap.height)
    const pixels = new Uint8ClampedArray(bitmap.width * bitmap.height)
    for (let i = 0; i < pixels.length; i++) {
      const offset = i * 4
      // Luma keeps RGB PNGs useful while preserving exact greyscale values.
      pixels[i] = Math.round(image.data[offset]! * 0.2126 + image.data[offset + 1]! * 0.7152 + image.data[offset + 2]! * 0.0722)
    }
    return { pixels, width: bitmap.width, height: bitmap.height }
  } finally {
    bitmap.close()
  }
}

/** Async counterpart used by image-backed terrain handles. */
export async function generateHeightmapAsync(map: MapDefinition): Promise<Float32Array> {
  if (!map.terrain.heightmap) return generateHeightmap(map)
  const decoded = await loadImageHeightmap(map.terrain.heightmap, { heightScale: map.terrain.heightScale })
  const source = normalizeHeightmapPixels(decoded.pixels, decoded.width, map.terrain.heightScale)
  const targetSize = map.segments + 1
  const output = new Float32Array(targetSize * targetSize)
  for (let z = 0; z < targetSize; z++) {
    const sourceZ = z / (targetSize - 1) * (decoded.height - 1)
    const z0 = Math.floor(sourceZ)
    const z1 = Math.min(z0 + 1, decoded.height - 1)
    const fz = sourceZ - z0
    for (let x = 0; x < targetSize; x++) {
      const sourceX = x / (targetSize - 1) * (decoded.width - 1)
      const x0 = Math.floor(sourceX)
      const x1 = Math.min(x0 + 1, decoded.width - 1)
      const fx = sourceX - x0
      const a = source[z0 * decoded.width + x0]!
      const b = source[z0 * decoded.width + x1]!
      const c = source[z1 * decoded.width + x0]!
      const d = source[z1 * decoded.width + x1]!
      output[z * targetSize + x] = a + (b - a) * fx + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fz
    }
  }
  return output
}

function createDocumentCanvas(width: number, height: number): HTMLCanvasElement {
  if (typeof document === 'undefined') throw new Error('Image heightmaps require a browser canvas')
  const canvas = document.createElement('canvas')
  canvas.width = width
  canvas.height = height
  return canvas
}

function generateProceduralHeightmap(map: MapDefinition): Float32Array {
  const cfg = map.terrain.procedural!
  const segW = map.segments
  const segD = map.segments
  const width = map.size.width
  const depth = map.size.depth
  const heightScale = map.terrain.heightScale
  const data = new Float32Array((segW + 1) * (segD + 1))

  for (let iz = 0; iz <= segD; iz++) {
    for (let ix = 0; ix <= segW; ix++) {
      // Map grid cell to world coordinates (centered at origin)
      const wx = (ix / segW - 0.5) * width
      const wz = (iz / segD - 0.5) * depth
      const h = sampleNoise(wx, wz, cfg) * heightScale * 0.12
      data[iz * (segW + 1) + ix] = h
    }
  }

  return data
}

/**
 * Sample the procedural noise at world (x, z).
 * Combines fractal noise with additional wave terms.
 */
function sampleNoise(x: number, z: number, cfg: ProceduralConfig): number {
  // Fractal noise via stacked sine waves (deterministic, no external deps)
  let value = 0
  let amplitude = 1.0
  let frequency = cfg.frequency
  for (let octave = 0; octave < cfg.octaves; octave++) {
    // Hash-based phase offset per octave (deterministic from seed)
    const phase = pseudoRandom(cfg.seed + octave * 137) * Math.PI * 2
    value += amplitude * (
      Math.sin(x * frequency + phase) * Math.cos(z * frequency * 0.8 + phase * 0.7)
    )
    frequency *= cfg.lacunarity
    amplitude *= cfg.gain
  }

  // Additional wave terms for terrain variety
  if (cfg.waves) {
    for (const wave of cfg.waves) {
      value += sampleWave(x, z, wave)
    }
  }

  return value
}

function sampleWave(x: number, z: number, wave: WaveTerm): number {
  const fx = wave.freqMul
  const ph = wave.phase

  switch (wave.op) {
    case 'sin':
      if (wave.axis === 'x') return wave.amplitude * Math.sin(x * fx + ph)
      if (wave.axis === 'z') return wave.amplitude * Math.sin(z * fx + ph)
      // 'xz' diagonal
      return wave.amplitude * Math.sin((x + z) * fx + ph)

    case 'cos':
      if (wave.axis === 'x') return wave.amplitude * Math.cos(x * fx + ph)
      if (wave.axis === 'z') return wave.amplitude * Math.cos(z * fx + ph)
      return wave.amplitude * Math.cos((x + z) * fx + ph)

    case 'product':
      // sin(x) * cos(z) style interaction
      if (wave.axis === 'xz') {
        return wave.amplitude * Math.sin(x * fx + ph) * Math.cos(z * fx * 0.7 + ph)
      }
      return wave.amplitude * Math.sin(x * fx + ph) * Math.cos(z * fx + ph)

    default:
      return 0
  }
}

/** Deterministic pseudo-random in [0, 1] from an integer seed. */
function pseudoRandom(seed: number): number {
  const x = Math.sin(seed * 127.1 + 311.7) * 43758.5453
  return x - Math.floor(x)
}
