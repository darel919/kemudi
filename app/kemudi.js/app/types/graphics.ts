export type GraphicsPreset = 'auto' | 'low' | 'medium' | 'high'

export interface PresetSettings {
  pixelRatioCap: number
  antialias: boolean
  shadows: boolean
  toneMappingExposure: number
  shadowMapSize: number
  textureSizeLimit: number
  vegetationDensity: number
  particleBudget: number
  postProcessing: boolean
  viewDistance: number
  vehicleLODDistance: number
  physicsSubsteps: number
  physicsIterations: number
  networkSnapshotRate: number
}

export const GRAPHICS_PRESETS: Record<Exclude<GraphicsPreset, 'auto'>, PresetSettings> = {
  low: {
    pixelRatioCap: 1,
    antialias: false,
    shadows: false,
    toneMappingExposure: 0.8,
    shadowMapSize: 256,
    textureSizeLimit: 512,
    vegetationDensity: 0,
    particleBudget: 50,
    postProcessing: false,
    viewDistance: 80,
    vehicleLODDistance: 20,
    physicsSubsteps: 2,
    physicsIterations: 4,
    networkSnapshotRate: 10,
  },
  medium: {
    pixelRatioCap: 1.5,
    antialias: true,
    shadows: true,
    toneMappingExposure: 1.0,
    shadowMapSize: 512,
    textureSizeLimit: 1024,
    vegetationDensity: 0.5,
    particleBudget: 200,
    postProcessing: false,
    viewDistance: 150,
    vehicleLODDistance: 50,
    physicsSubsteps: 4,
    physicsIterations: 6,
    networkSnapshotRate: 20,
  },
  high: {
    pixelRatioCap: 2,
    antialias: true,
    shadows: true,
    toneMappingExposure: 1.2,
    shadowMapSize: 1024,
    textureSizeLimit: 2048,
    vegetationDensity: 1,
    particleBudget: 500,
    postProcessing: true,
    viewDistance: 300,
    vehicleLODDistance: 100,
    physicsSubsteps: 6,
    physicsIterations: 8,
    networkSnapshotRate: 30,
  },
}

export const DEFAULT_PRESET: GraphicsPreset = 'auto'

export interface GraphicsCapabilities {
  webgl2: boolean
  wasm: boolean
  maxTextureSize: number
  deviceMemory: number
  hardwareConcurrency: number
}

export function getEffectivePreset(userPreset: GraphicsPreset): Exclude<GraphicsPreset, 'auto'> {
  if (userPreset !== 'auto') return userPreset
  return detectAutoPreset()
}

export function detectAutoPreset(): Exclude<GraphicsPreset, 'auto'> {
  if (typeof window === 'undefined') return 'medium'

  let gl: WebGL2RenderingContext | null = null
  try {
    gl = document.createElement('canvas').getContext('webgl2')
  } catch {
    gl = null
  }
  const isWebGL2 = !!gl
  const hasWasm = typeof WebAssembly !== 'undefined'
  if (!isWebGL2 || !hasWasm) return 'low'

  const maxTextureSize = gl?.getParameter(gl.MAX_TEXTURE_SIZE) ?? 2048
  const deviceMemory = (navigator as any).deviceMemory ?? 4
  const hardwareConcurrency = navigator.hardwareConcurrency ?? 4
  const dpr = window.devicePixelRatio || 1

  let score = 0
  if (isWebGL2) score += 2
  if (maxTextureSize >= 4096) score += 2
  else if (maxTextureSize >= 2048) score += 1
  if (deviceMemory >= 8) score += 2
  else if (deviceMemory >= 4) score += 1
  if (hardwareConcurrency >= 8) score += 2
  else if (hardwareConcurrency >= 4) score += 1
  if (dpr <= 1.5) score += 1

  if (score >= 7) return 'high'
  if (score >= 4) return 'medium'
  return 'low'
}

export function detectGraphicsCapabilities(): GraphicsCapabilities {
  if (typeof window === 'undefined') {
    return { webgl2: false, wasm: false, maxTextureSize: 0, deviceMemory: 0, hardwareConcurrency: 0 }
  }
  let gl: WebGL2RenderingContext | null = null
  try {
    gl = document.createElement('canvas').getContext('webgl2')
  } catch {
    gl = null
  }
  return {
    webgl2: gl !== null,
    wasm: typeof WebAssembly !== 'undefined',
    maxTextureSize: gl?.getParameter(gl.MAX_TEXTURE_SIZE) ?? 0,
    deviceMemory: Number((navigator as Navigator & { deviceMemory?: number }).deviceMemory ?? 0),
    hardwareConcurrency: navigator.hardwareConcurrency ?? 0,
  }
}

export function getPresetSettings(preset: GraphicsPreset): PresetSettings {
  const effective = getEffectivePreset(preset)
  return GRAPHICS_PRESETS[effective]
}

export function validatePreset(value: unknown): value is GraphicsPreset {
  return ['auto', 'low', 'medium', 'high'].includes(value as string)
}
