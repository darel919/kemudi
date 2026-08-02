/**
 * Map loader — fetches MapDefinition from public/maps/<id>/map.json.
 *
 * To add a new map:
 *   1. Create public/maps/<new-id>/map.json following MapDefinition schema
 *   2. Add the id to KNOWN_MAP_IDS below
 *   3. (Optional) Add a heightmap.png in the same directory
 */

import type { MapDefinition } from '~/types/map-schema'

/** All known map IDs. Add new maps here. */
export const KNOWN_MAP_IDS = ['ground-zero', 'dragville', 'amazon'] as const
export type BaseMapId = (typeof KNOWN_MAP_IDS)[number]

export type TerrainProfile = BaseMapId

/** In-memory cache populated by loadAllMaps(). */
let cachedMaps: Record<BaseMapId, MapDefinition> | null = null

/**
 * Fetch and validate every map from public/maps/<id>/map.json.
 * Returns a record keyed by BaseMapId. Throws on missing or invalid file.
 */
export async function loadAllMaps(): Promise<Record<BaseMapId, MapDefinition>> {
  if (cachedMaps) return cachedMaps

  const entries = await Promise.all(
    KNOWN_MAP_IDS.map(async (id) => {
      const res = await fetch(`/maps/${id}/map.json`)
      if (!res.ok) throw new Error(`Failed to load map "${id}": ${res.status}`)
      const raw = await res.json()
      const map = validateMap(raw)
      return [map.id, map] as const
    }),
  )

  cachedMaps = Object.fromEntries(entries) as Record<BaseMapId, MapDefinition>
  return cachedMaps
}

/** Fetch a single map by ID. */
export async function loadMap(id: BaseMapId): Promise<MapDefinition> {
  const all = await loadAllMaps()
  return all[id]
}

/** Validate raw JSON into a MapDefinition. Throws on missing required fields. */
function validateMap(raw: Record<string, unknown>): MapDefinition {
  const id = String(raw.id ?? '')
  if (!KNOWN_MAP_IDS.includes(id as BaseMapId)) {
    throw new Error(`Unknown map id: "${id}". Expected one of: ${KNOWN_MAP_IDS.join(', ')}`)
  }

  const size = raw.size as Record<string, unknown> | undefined
  if (!size || typeof size.width !== 'number' || typeof size.depth !== 'number') {
    throw new Error(`Map "${id}" missing valid size.width / size.depth`)
  }

  const terrain = raw.terrain as Record<string, unknown> | undefined
  if (!terrain) throw new Error(`Map "${id}" missing terrain config`)

  return {
    id: id as BaseMapId,
    label: String(raw.label ?? id),
    description: String(raw.description ?? ''),
    version: Number(raw.version ?? 1),
    size: { width: Number(size.width), depth: Number(size.depth) },
    segments: Number(raw.segments ?? 64),
    preview: {
      bgColor: Number((raw.preview as Record<string, unknown>)?.bgColor ?? 0x3b4650),
      pattern: String((raw.preview as Record<string, unknown>)?.pattern ?? 'none') as MapDefinition['preview']['pattern'],
      patternColor: (raw.preview as Record<string, unknown>)?.patternColor != null ? Number((raw.preview as Record<string, unknown>).patternColor) : undefined,
      patternOpacity: (raw.preview as Record<string, unknown>)?.patternOpacity != null ? Number((raw.preview as Record<string, unknown>).patternOpacity) : undefined,
      stripColor: (raw.preview as Record<string, unknown>)?.stripColor != null ? String((raw.preview as Record<string, unknown>).stripColor) : undefined,
      stripAngle: (raw.preview as Record<string, unknown>)?.stripAngle != null ? Number((raw.preview as Record<string, unknown>).stripAngle) : undefined,
      stripWidth: (raw.preview as Record<string, unknown>)?.stripWidth != null ? String((raw.preview as Record<string, unknown>).stripWidth) : undefined,
    },
    terrain: {
      heightmap: (terrain.heightmap as string) ?? null,
      heightScale: Number(terrain.heightScale ?? 0),
      color: Number(terrain.color ?? 0x3b4650),
      roughness: Number(terrain.roughness ?? 0.76),
      groundFriction: Number(terrain.groundFriction ?? 0.94),
      procedural: terrain.procedural as MapDefinition['terrain']['procedural'] ?? null,
      layers: Array.isArray(terrain.layers) ? terrain.layers as MapDefinition['terrain']['layers'] : [],
    },
    roads: Array.isArray(raw.roads) ? raw.roads as MapDefinition['roads'] : [],
    objects: Array.isArray(raw.objects) ? raw.objects as MapDefinition['objects'] : [],
    spawnPoints: Array.isArray(raw.spawnPoints) ? raw.spawnPoints as MapDefinition['spawnPoints'] : [],
  }
}

/**
 * Map a TerrainProfile string to the numeric ID the Rust WASM expects.
 * 0 = ground-zero, 1 = dragville, 2 = amazon
 */
export function terrainProfileToId(profile: TerrainProfile): 0 | 1 | 2 {
  if (profile === 'dragville') return 1
  if (profile === 'amazon') return 2
  return 0
}

/** Get the numeric profile from a MapDefinition. */
export function mapToTerrainProfile(map: MapDefinition): 0 | 1 | 2 {
  return terrainProfileToId(map.id as BaseMapId)
}
