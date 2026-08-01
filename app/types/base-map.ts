export type BaseMapId = 'flat' | 'bumpy' | 'offroad'

export type TerrainProfile = 'flat' | 'bumpy' | 'offroad'

export interface BaseMapDefinition {
  id: BaseMapId
  label: string
  description: string
  profile: TerrainProfile
  surfaceLabel: string
  color: number
  roughness: number
  friction: number
  width: number
  depth: number
  segments: number
  heightScale: number
}

export const BASE_MAPS: Record<BaseMapId, BaseMapDefinition> = {
  flat: {
    id: 'flat',
    label: 'Test Pad',
    description: 'A perfectly flat surface for tuning and control tests.',
    profile: 'flat',
    surfaceLabel: 'FLAT ASPHALT',
    color: 0x3b4650,
    roughness: 0.76,
    friction: 1.0,
    width: 400,
    depth: 400,
    segments: 32,
    heightScale: 0,
  },
  bumpy: {
    id: 'bumpy',
    label: 'Bumpy Road',
    description: 'A paved route with repeating bumps and shallow undulations.',
    profile: 'bumpy',
    surfaceLabel: 'BUMPY ASPHALT',
    color: 0x46534a,
    roughness: 0.88,
    friction: 0.86,
    width: 400,
    depth: 400,
    segments: 96,
    heightScale: 7,
  },
  offroad: {
    id: 'offroad',
    label: 'Offroad Range',
    description: 'Uneven dirt terrain with larger ridges and reduced grip.',
    profile: 'offroad',
    surfaceLabel: 'LOOSE DIRT',
    color: 0x806347,
    roughness: 1,
    friction: 0.62,
    width: 400,
    depth: 400,
    segments: 112,
    heightScale: 13,
  },
}

export function getBaseMap(id: BaseMapId): BaseMapDefinition {
  return BASE_MAPS[id] ?? BASE_MAPS.flat
}
