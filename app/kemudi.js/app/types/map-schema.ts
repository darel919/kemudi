/**
 * Map definition schema
 *
 * EVERY visual and physical element is described as data in the map file.
 * The engine is a generic renderer — it reads this schema and creates meshes.
 * No map-type conditionals in engine code.
 *
 * Map directory: public/maps/<id>/
 *   map.json      — this schema
 *   heightmap.png — (optional) 8-bit grayscale PNG for terrain elevation
 */

export interface MapDefinition {
  id: string
  label: string
  description: string
  version: number
  size: { width: number; depth: number }
  segments: number
  /** Preview card styling for the main menu. */
  preview: PreviewConfig
  terrain: TerrainConfig
  /** Optional explicit collision walls. Missing values use generic map-edge walls. */
  boundaries?: BoundaryDefinition[]
  /** Optional named surface materials referenced by terrain layers and roads. */
  surfaces?: SurfaceMaterialDefinition[]
  roads: RoadDefinition[]
  objects: MapObject[]
  spawnPoints: SpawnPoint[]
}

/* ── Preview ───────────────────────────────────────────────────────────── */

/** Data-driven preview card for the main menu. No CSS per map — all from here. */
export interface PreviewConfig {
  /** Background color of the preview card. */
  bgColor: number
  /** Optional secondary background color for the road/track strip overlay. */
  trackColor?: number
  /** Background pattern type. */
  pattern: 'grid' | 'stripe' | 'dots' | 'none'
  /** Pattern element color. */
  patternColor?: number
  /** Pattern element opacity (0-1). */
  patternOpacity?: number
  /** Rotation angle of the track strip in degrees. */
  stripAngle?: number
  /** Width of the track strip overlay (CSS %). */
  stripWidth?: string
  /** Strip background color. */
  stripColor?: string
}

/* ── Terrain ───────────────────────────────────────────────────────────── */

export interface TerrainConfig {
  heightmap: string | null
  heightScale: number
  color: number
  roughness: number
  /** Base ground friction for chassis-terrain collision. Set from map data. */
  groundFriction: number
  procedural: ProceduralConfig | null
  layers: TerrainLayer[]
}

export interface ProceduralConfig {
  frequency: number
  octaves: number
  lacunarity: number
  gain: number
  seed: number
  waves: WaveTerm[]
}

export interface WaveTerm {
  amplitude: number
  freqMul: number
  phase: number
  axis: 'x' | 'z' | 'xz'
  op: 'sin' | 'cos' | 'product'
}

export interface TerrainLayer {
  name: string
  surfaceId: number
  color: number
  slopeRange: [number, number]
  heightRange: [number, number]
  priority: number
}

/* ── Roads ─────────────────────────────────────────────────────────────── */

export interface RoadDefinition {
  name: string
  width: number
  surfaceId: number
  /** Optional per-road overrides applied at physics contact points. */
  surface?: SurfaceMetadata
  points: Array<{ x: number; z: number }>
  visible: boolean
}

export interface SurfaceMetadata {
  friction: number
  roughness: number
  moisture: number
  compactness: number
}

export interface SurfaceMaterialDefinition extends SurfaceMetadata {
  id: number
  name: string
}

export interface BoundaryDefinition {
  name?: string
  position: { x: number; y?: number; z: number }
  size: [number, number, number]
  rotation?: { x?: number; y?: number; z?: number }
  restitution?: number
  friction?: number
}

/* ── Objects ───────────────────────────────────────────────────────────── */

export interface MapObject {
  type: ObjectType
  placement: Placement
  visual: VisualProperties
  /** Optional authored physics shape; box/sphere visuals get a safe default. */
  collision?: CollisionShape
  name?: string
}

export type CollisionShape =
  | { type: 'box'; size: [number, number, number]; offset?: { x?: number; y?: number; z?: number }; restitution?: number; friction?: number }
  | { type: 'sphere'; radius: number; offset?: { x?: number; y?: number; z?: number }; restitution?: number; friction?: number }

export type ObjectType =
  | 'box'
  | 'sphere'
  | 'circle'
  | 'line'
  | 'plane'

export type Placement =
  | InstancePlacement
  | ScatterPlacement
  | SequencePlacement
  | SplinePlacement

export interface InstancePlacement {
  mode: 'instance'
  position: { x: number; y?: number; z: number }
  rotation?: { x: number; y: number; z: number }
}

export interface ScatterPlacement {
  mode: 'scatter'
  count: number
  seed: number
  spread: number
  scale?: { min: number; max: number }
}

export interface SequencePlacement {
  mode: 'sequence'
  axis: 'x' | 'z'
  start: number
  end: number
  step: number
  fixedCoord: number
  terrainOffset?: number
  y?: number
}

export interface SplinePlacement {
  mode: 'spline'
  roadName: string
  spacing: number
  offset: number
  terrainOffset?: number
}

export interface VisualProperties {
  color?: number
  opacity?: number
  roughness?: number
  metalness?: number
  flatShading?: boolean
  castShadow?: boolean
  receiveShadow?: boolean
  visible?: boolean
  size?: [number, number, number]
  radius?: number
  circleRadius?: number
  linePoints?: number[]
}

/* ── Spawn ─────────────────────────────────────────────────────────────── */

export interface SpawnPoint {
  x: number
  z: number
  heading: number
  label?: string
}
