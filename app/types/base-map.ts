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

/** Validate raw JSON into a bounded MapDefinition. */
export function validateMap(raw: unknown): MapDefinition {
  const source = asRecord(raw, 'Map must be an object')
  const id = stringValue(source.id, 'id')
  if (!KNOWN_MAP_IDS.includes(id as BaseMapId)) {
    throw new Error(`Unknown map id: "${id}". Expected one of: ${KNOWN_MAP_IDS.join(', ')}`)
  }

  const size = asRecord(source.size, `Map "${id}" missing size`)
  const width = finiteBounded(size.width, `Map "${id}" size.width`, 1, 100_000)
  const depth = finiteBounded(size.depth, `Map "${id}" size.depth`, 1, 100_000)
  const segments = integerBounded(source.segments ?? 64, `Map "${id}" segments`, 1, 1024)
  const terrain = asRecord(source.terrain, `Map "${id}" missing terrain config`)
  const surfaces = parseSurfaces(source.surfaces, id)
  const roads = parseRoads(source.roads, surfaces, id)
  const objects = parseObjects(source.objects, roads, id)
  for (const [index, road] of roads.entries()) {
    if (road.points.length < 2) throw new Error(`Map "${id}" road ${index} needs 2-4096 points`)
  }
  const boundaries = parseBoundaries(source.boundaries, width, depth, id)
  const preview = asRecord(source.preview ?? {}, `Map "${id}" preview`)

  return {
    id: id as BaseMapId,
    label: stringValue(source.label ?? id, 'label'),
    description: stringValue(source.description ?? '', 'description'),
    version: finiteBounded(source.version ?? 1, `Map "${id}" version`, 1, 10_000),
    size: { width, depth },
    segments,
    preview: {
      bgColor: finiteBounded(preview.bgColor ?? 0x3b4650, 'preview.bgColor', 0, 0xffffff),
      pattern: enumValue(preview.pattern ?? 'none', ['grid', 'stripe', 'dots', 'none'] as const, 'preview.pattern'),
      patternColor: optionalFinite(preview.patternColor, 'preview.patternColor', 0, 0xffffff),
      patternOpacity: optionalFinite(preview.patternOpacity, 'preview.patternOpacity', 0, 1),
      stripColor: preview.stripColor == null ? undefined : stringValue(preview.stripColor, 'preview.stripColor'),
      stripAngle: optionalFinite(preview.stripAngle, 'preview.stripAngle', -360, 360),
      stripWidth: preview.stripWidth == null ? undefined : stringValue(preview.stripWidth, 'preview.stripWidth'),
    },
    terrain: parseTerrain(terrain, id),
    boundaries,
    surfaces: surfaces.length > 0 ? surfaces : undefined,
    roads,
    objects,
    spawnPoints: parseSpawnPoints(source.spawnPoints, width, depth, id),
  }
}

const BUILTIN_SURFACE_IDS = new Set([0, 1, 2, 3, 4, 5, 6, 7])

type RecordValue = Record<string, unknown>
function asRecord(value: unknown, message: string): RecordValue {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new Error(message)
  return value as RecordValue
}
function stringValue(value: unknown, field: string): string {
  if (typeof value !== 'string' || value.length > 512) throw new Error(`${field} must be a bounded string`)
  return value
}
function finiteBounded(value: unknown, field: string, min: number, max: number): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error(`${field} must be finite`)
  if (value < min || value > max) throw new Error(`${field} must be between ${min} and ${max}`)
  return value
}
function optionalFinite(value: unknown, field: string, min: number, max: number): number | undefined {
  return value == null ? undefined : finiteBounded(value, field, min, max)
}
function integerBounded(value: unknown, field: string, min: number, max: number): number {
  const result = finiteBounded(value, field, min, max)
  if (!Number.isInteger(result)) throw new Error(`${field} must be an integer`)
  return result
}
function enumValue<T extends readonly string[]>(value: unknown, values: T, field: string): T[number] {
  if (typeof value !== 'string' || !values.includes(value)) throw new Error(`${field} has an unknown value`)
  return value as T[number]
}
function finiteCoordinate(value: unknown, field: string): number {
  return finiteBounded(value, field, -100_000, 100_000)
}
function vector3(value: unknown, field: string): { x: number; y?: number; z: number } {
  const point = asRecord(value, `${field} must be an object`)
  return { x: finiteCoordinate(point.x, `${field}.x`), y: point.y == null ? undefined : finiteCoordinate(point.y, `${field}.y`), z: finiteCoordinate(point.z, `${field}.z`) }
}
function size3(value: unknown, field: string): [number, number, number] {
  if (!Array.isArray(value) || value.length !== 3) throw new Error(`${field} must contain three values`)
  return [1, 2, 3].map((_, index) => finiteBounded(value[index], `${field}[${index}]`, 0.001, 100_000)) as [number, number, number]
}
function parseSurfaceMetadata(value: unknown, field: string) {
  const source = asRecord(value ?? {}, field)
  return {
    friction: finiteBounded(source.friction ?? 0.8, `${field}.friction`, 0, 2),
    roughness: finiteBounded(source.roughness ?? 0.65, `${field}.roughness`, 0, 1),
    moisture: finiteBounded(source.moisture ?? 0, `${field}.moisture`, 0, 1),
    compactness: finiteBounded(source.compactness ?? 1, `${field}.compactness`, 0, 1),
  }
}
function parseSurfaces(value: unknown, id: string) {
  if (value == null) return []
  if (!Array.isArray(value) || value.length > 256) throw new Error(`Map "${id}" surfaces must be a bounded array`)
  const seen = new Set<number>()
  return value.map((entry, index) => {
    const source = asRecord(entry, `Map "${id}" surface ${index} is malformed`)
    const surfaceId = integerBounded(source.id, `surface ${index}.id`, 0, 255)
    if (seen.has(surfaceId)) throw new Error(`Map "${id}" duplicates surface id ${surfaceId}`)
    seen.add(surfaceId)
    return { id: surfaceId, name: stringValue(source.name ?? `surface-${surfaceId}`, `surface ${index}.name`), ...parseSurfaceMetadata(source, `surface ${index}`) }
  })
}
function validateSurfaceId(value: unknown, surfaces: readonly { id: number }[], field: string): number {
  const surfaceId = integerBounded(value ?? 0, field, 0, 255)
  if (surfaces.length > 0 && !surfaces.some(surface => surface.id === surfaceId) && !BUILTIN_SURFACE_IDS.has(surfaceId)) {
    throw new Error(`${field} references unknown surface ${surfaceId}`)
  }
  return surfaceId
}
function parseTerrain(source: RecordValue, id: string): MapDefinition['terrain'] {
  const procedural = source.procedural == null ? null : source.procedural as MapDefinition['terrain']['procedural']
  return {
    heightmap: source.heightmap == null ? null : stringValue(source.heightmap, `Map "${id}" terrain.heightmap`),
    heightScale: finiteBounded(source.heightScale ?? 0, 'terrain.heightScale', 0, 10_000),
    color: finiteBounded(source.color ?? 0x3b4650, 'terrain.color', 0, 0xffffff),
    roughness: finiteBounded(source.roughness ?? 0.76, 'terrain.roughness', 0, 1),
    groundFriction: finiteBounded(source.groundFriction ?? 0.94, 'terrain.groundFriction', 0, 1.5),
    procedural,
    layers: Array.isArray(source.layers) ? source.layers as MapDefinition['terrain']['layers'] : [],
  }
}
function parseRoads(value: unknown, surfaces: readonly { id: number }[], id: string): MapDefinition['roads'] {
  if (value == null) return []
  if (!Array.isArray(value) || value.length > 1024) throw new Error(`Map "${id}" roads must be a bounded array`)
  return value.map((entry, index) => {
    const source = asRecord(entry, `Map "${id}" road ${index} is malformed`)
    const points = source.points
    if (!Array.isArray(points) || points.length < 1 || points.length > 4096) throw new Error(`Map "${id}" road ${index} needs 1-4096 points`)
    return {
      name: stringValue(source.name, `road ${index}.name`),
      width: finiteBounded(source.width, `road ${index}.width`, 0.01, 10_000),
      surfaceId: validateSurfaceId(source.surfaceId, surfaces, `road ${index}.surfaceId`),
      surface: source.surface == null ? undefined : parseSurfaceMetadata(source.surface, `road ${index}.surface`),
      points: points.map((point, pointIndex) => { const p = asRecord(point, `road ${index}.point ${pointIndex}`); return { x: finiteCoordinate(p.x, `road ${index}.point.x`), z: finiteCoordinate(p.z, `road ${index}.point.z`) } }),
      visible: source.visible !== false,
    }
  })
}
function parseObjects(value: unknown, roads: readonly { name: string }[], id: string): MapDefinition['objects'] {
  if (value == null) return []
  if (!Array.isArray(value) || value.length > 4096) throw new Error(`Map "${id}" objects must be a bounded array`)
  return value.map((entry, index) => {
    const source = asRecord(entry, `Map "${id}" object ${index} is malformed`)
    const type = enumValue(source.type, ['box', 'sphere', 'circle', 'line', 'plane'] as const, `object ${index}.type`)
    const placement = parsePlacement(source.placement, roads, `object ${index}.placement`)
    const visual = asRecord(source.visual ?? {}, `object ${index}.visual`) as MapDefinition['objects'][number]['visual']
    const collision = source.collision == null ? defaultCollision(type, visual) : parseCollision(source.collision, `object ${index}.collision`)
    return { type, placement, visual, collision, name: source.name == null ? undefined : stringValue(source.name, `object ${index}.name`) }
  })
}
function defaultCollision(type: MapDefinition['objects'][number]['type'], visual: MapDefinition['objects'][number]['visual']): MapDefinition['objects'][number]['collision'] {
  if (type === 'box') return { type: 'box', size: size3(visual.size ?? [1, 1, 1], 'visual.size') }
  if (type === 'sphere') return { type: 'sphere', radius: finiteBounded(visual.radius ?? 1, 'visual.radius', 0.01, 1000) }
  return undefined
}
function parseCollision(value: unknown, field: string): MapDefinition['objects'][number]['collision'] {
  const source = asRecord(value, field)
  const type = enumValue(source.type, ['box', 'sphere'] as const, `${field}.type`)
  const offset = source.offset == null ? undefined : vector3({ ...asRecord(source.offset, `${field}.offset`), y: (asRecord(source.offset, `${field}.offset`).y ?? 0) }, `${field}.offset`)
  return type === 'box'
    ? { type, size: size3(source.size, `${field}.size`), offset, restitution: optionalFinite(source.restitution, `${field}.restitution`, 0, 1), friction: optionalFinite(source.friction, `${field}.friction`, 0, 1) }
    : { type, radius: finiteBounded(source.radius, `${field}.radius`, 0.01, 1000), offset, restitution: optionalFinite(source.restitution, `${field}.restitution`, 0, 1), friction: optionalFinite(source.friction, `${field}.friction`, 0, 1) }
}
function parsePlacement(value: unknown, roads: readonly { name: string }[], field: string): MapDefinition['objects'][number]['placement'] {
  const source = asRecord(value, `${field} is malformed`)
  const mode = enumValue(source.mode, ['instance', 'scatter', 'sequence', 'spline'] as const, `${field}.mode`)
  if (mode === 'instance') return { mode, position: vector3(source.position, `${field}.position`), rotation: source.rotation == null ? undefined : vector3(source.rotation, `${field}.rotation`) as { x: number; y: number; z: number } }
  if (mode === 'scatter') return { mode, count: integerBounded(source.count, `${field}.count`, 0, 100_000), seed: integerBounded(source.seed ?? 0, `${field}.seed`, 0, 2_147_483_647), spread: finiteBounded(source.spread, `${field}.spread`, 0, 1), scale: source.scale == null ? undefined : { min: finiteBounded(asRecord(source.scale, `${field}.scale`).min, `${field}.scale.min`, 0.01, 1000), max: finiteBounded(asRecord(source.scale, `${field}.scale`).max, `${field}.scale.max`, 0.01, 1000) } }
  if (mode === 'sequence') return { mode, axis: enumValue(source.axis, ['x', 'z'] as const, `${field}.axis`), start: finiteCoordinate(source.start, `${field}.start`), end: finiteCoordinate(source.end, `${field}.end`), step: finiteBounded(source.step, `${field}.step`, 0.001, 100_000), fixedCoord: finiteCoordinate(source.fixedCoord, `${field}.fixedCoord`), terrainOffset: optionalFinite(source.terrainOffset, `${field}.terrainOffset`, -1000, 1000), y: optionalFinite(source.y, `${field}.y`, -1000, 1000) }
  const roadName = stringValue(source.roadName, `${field}.roadName`)
  if (!roads.some(road => road.name === roadName)) throw new Error(`${field} references unknown road "${roadName}"`)
  return { mode, roadName, spacing: finiteBounded(source.spacing, `${field}.spacing`, 0.001, 1000), offset: finiteCoordinate(source.offset, `${field}.offset`), terrainOffset: optionalFinite(source.terrainOffset, `${field}.terrainOffset`, -1000, 1000) }
}
function parseBoundaries(value: unknown, width: number, depth: number, id: string): NonNullable<MapDefinition['boundaries']> {
  if (value == null) {
    const wallHeight = 20
    const thickness = 1
    return [
      { name: 'west', position: { x: -width / 2, y: wallHeight / 2, z: 0 }, size: [thickness, wallHeight, depth + thickness * 2] },
      { name: 'east', position: { x: width / 2, y: wallHeight / 2, z: 0 }, size: [thickness, wallHeight, depth + thickness * 2] },
      { name: 'north', position: { x: 0, y: wallHeight / 2, z: -depth / 2 }, size: [width + thickness * 2, wallHeight, thickness] },
      { name: 'south', position: { x: 0, y: wallHeight / 2, z: depth / 2 }, size: [width + thickness * 2, wallHeight, thickness] },
    ]
  }
  if (!Array.isArray(value) || value.length > 64) throw new Error(`Map "${id}" boundaries must be a bounded array`)
  return value.map((entry, index) => { const source = asRecord(entry, `boundary ${index}`); return { name: source.name == null ? undefined : stringValue(source.name, `boundary ${index}.name`), position: vector3(source.position, `boundary ${index}.position`), size: size3(source.size, `boundary ${index}.size`), rotation: source.rotation == null ? undefined : vector3({ ...asRecord(source.rotation, `boundary ${index}.rotation`), y: asRecord(source.rotation, `boundary ${index}.rotation`).y ?? 0 }, `boundary ${index}.rotation`) as { x?: number; y?: number; z?: number }, restitution: optionalFinite(source.restitution, `boundary ${index}.restitution`, 0, 1), friction: optionalFinite(source.friction, `boundary ${index}.friction`, 0, 1) } })
}
function parseSpawnPoints(value: unknown, width: number, depth: number, id: string): MapDefinition['spawnPoints'] {
  if (value == null) return []
  if (!Array.isArray(value) || value.length > 256) throw new Error(`Map "${id}" spawnPoints must be a bounded array`)
  return value.map((entry, index) => { const source = asRecord(entry, `spawn point ${index}`); const x = finiteCoordinate(source.x, `spawn point ${index}.x`); const z = finiteCoordinate(source.z, `spawn point ${index}.z`); if (Math.abs(x) > width / 2 || Math.abs(z) > depth / 2) throw new Error(`spawn point ${index} lies outside map bounds`); return { x, z, heading: finiteCoordinate(source.heading, `spawn point ${index}.heading`), label: source.label == null ? undefined : stringValue(source.label, `spawn point ${index}.label`) } })
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

/**
 * Select the map's default built-in physics material from its lowest-priority
 * terrain layer. The Rust contact model currently exposes ten built-in
 * presets, so custom material IDs are conservatively mapped to asphalt until
 * the versioned material catalog crosses the worker boundary.
 */
export function mapToSurfacePresetIndex(map: MapDefinition): number {
  const baseLayer = [...map.terrain.layers]
    .sort((left, right) => left.priority - right.priority)[0]
  const surfaceId = baseLayer?.surfaceId ?? 0
  return Math.max(0, Math.min(9, Math.trunc(surfaceId)))
}
