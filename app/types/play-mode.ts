export type PlayModeId = 'freeroam' | 'drag-race'

export type PlayModeLifecycle =
  | 'idle'
  | 'loading'
  | 'active'
  | 'paused'
  | 'ended'
  | 'error'

export type DragRacePhase =
  | 'none'
  | 'staging'
  | 'pre-stage'
  | 'stage'
  | 'countdown'
  | 'launch'
  | 'running'
  | 'finished'
  | 'timeout'
  | 'abandoned'
  | 'false-start'
  | 'disqualified'

export interface PlayModeRules {
  allowVehicleChange: boolean
  allowCameraChange: boolean
  allowTelemetryAccess: boolean
  hasTimer: boolean
  hasFinishCondition: boolean
}

export interface PlayModeDefinition {
  id: PlayModeId
  label: string
  description: string
  lifecycle: PlayModeLifecycle
  rules: PlayModeRules
  cameraDefaults: string
  dragRace?: DragRaceConfig
}

export interface DragRaceConfig {
  /** 1609.344 meters (1 mile) */
  trackLengthMeters: number
  laneCount: number
  countdownSeconds: number
  maxRunTimeSeconds: number
  stagingRequired: boolean
  /** Split distances in meters: [60ft, 330ft, ¼-mile, ½-mile, 1-mile] */
  splits: number[]
}

export interface DragRaceResult {
  reactionTime: number
  elapsedTime: number
  trapSpeed: number
  peakSpeed: number
  splits: { distance: number; time: number; speed: number }[]
  finishReason: 'completed' | 'timeout' | 'false-start' | 'disqualified' | 'abandoned'
  lane: number
  gearShifts: number
  maxWheelSlip: number
}

// Valid phase transitions: from -> Set<to>
export const DRAG_RACE_TRANSITIONS: Record<DragRacePhase, Set<DragRacePhase>> = {
  none:             new Set(['staging']),
  staging:          new Set(['pre-stage']),
  'pre-stage':      new Set(['stage']),
  stage:            new Set(['countdown']),
  countdown:        new Set(['launch', 'false-start']),
  launch:           new Set(['running', 'false-start']),
  running:          new Set(['finished', 'timeout', 'abandoned', 'disqualified']),
  finished:         new Set(['none']),
  timeout:          new Set(['none']),
  abandoned:        new Set(['none']),
  'false-start':    new Set(['none']),
  disqualified:     new Set(['none']),
}

/** Official drag strip split distances in meters */
export const DRAG_STRIP_SPLITS = [18.288, 100.584, 402.336, 804.672, 1609.344] as const

export const DRAG_RACE_CONFIG: DragRaceConfig = {
  trackLengthMeters: 1609.344,
  laneCount: 2,
  countdownSeconds: 3,
  maxRunTimeSeconds: 30,
  stagingRequired: true,
  splits: [...DRAG_STRIP_SPLITS],
}

export const PLAY_MODE_DEFINITIONS: Record<PlayModeId, PlayModeDefinition> = {
  freeroam: {
    id: 'freeroam',
    label: 'Freeroam',
    description: 'Open-world driving with no restrictions',
    lifecycle: 'idle',
    rules: {
      allowVehicleChange: true,
      allowCameraChange: true,
      allowTelemetryAccess: true,
      hasTimer: false,
      hasFinishCondition: false,
    },
    cameraDefaults: 'exterior',
  },
  'drag-race': {
    id: 'drag-race',
    label: 'Drag Race',
    description: 'Exactly one-mile drag strip competition',
    lifecycle: 'idle',
    rules: {
      allowVehicleChange: false,
      allowCameraChange: true,
      allowTelemetryAccess: true,
      hasTimer: true,
      hasFinishCondition: true,
    },
    cameraDefaults: 'exterior',
    dragRace: { ...DRAG_RACE_CONFIG },
  },
}
