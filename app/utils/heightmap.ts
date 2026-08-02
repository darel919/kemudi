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
    // Image-based heightmap loading — placeholder for future implementation.
    // When a heightmap.png is present, load and sample it here.
    throw new Error(`Image heightmap not yet implemented: ${map.terrain.heightmap}`)
  }

  if (map.terrain.procedural) {
    return generateProceduralHeightmap(map)
  }

  // Flat terrain (heightScale = 0, no procedural config)
  return new Float32Array((map.segments + 1) * (map.segments + 1))
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
