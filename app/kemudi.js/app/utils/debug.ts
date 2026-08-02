const STORAGE_KEY = 'kemudi:graphics-preset'

const DEBUG = import.meta.env.DEV

type LogScope =
  | 'graphics:preset-selected'
  | 'graphics:capability-detected'
  | 'renderer:initialized'
  | 'terrain:initialized'
  | 'terrain:heightmap-loaded'
  | 'renderer:frame-budget'
  | 'physics-worker:initialized'
  | 'physics-worker:error'
  | 'network:state-changed'
  | 'performance:budget-warning'
  | 'skinning:initialized'
  | 'skinning:disposed'
  | 'vehicle-mesh:mounted'
  | 'vehicle-mesh:disposed'
  | 'vehicle-mesh:unmounted'
  | 'vehicle-mesh:gltf-loaded'
  | 'vehicle-mesh:gltf-no-mesh'
  | 'vehicle-mesh:gltf-error'
  | 'vehicle-mesh:falling-back-to-box'
  | 'vehicle-loader:loaded'
  | 'play-mode:lifecycle'
  | 'play-mode:drag-race-phase'
  | 'play-mode:drag-race-result'
  | 'safety:abs'
  | 'safety:tc'
  | 'safety:vsc'
  | 'safety:adas'
  | 'engine:stress'

interface LogEntry {
  scope: LogScope
  data: Record<string, unknown>
}

// Queue up to 200 logs — silently drop oldest when full
const LOG_BUFFER: LogEntry[] = []
const MAX_LOG_BUFFER = 200

export function isDebugEnabled(): boolean {
  return DEBUG
}

function pushLog(scope: LogScope, data: Record<string, unknown>) {
  if (!isDebugEnabled()) return

  const entry = { scope, data }
  LOG_BUFFER.push(entry)
  if (LOG_BUFFER.length > MAX_LOG_BUFFER) {
    LOG_BUFFER.shift()
  }
  console.debug(`[${scope}]`, data)
}

export function logDebug(scope: LogScope, data: Record<string, unknown>) {
  pushLog(scope, data)
}

export function logThrottled(scope: LogScope, data: Record<string, unknown>, key: string, intervalMs = 2000) {
  if (!isDebugEnabled()) return
  // Simple throttle: skip if same key was logged within intervalMs
  const last = lastThrottle.get(key)
  const now = performance.now()
  if (last !== undefined && now - last < intervalMs) return
  lastThrottle.set(key, now)
  pushLog(scope, data)
}

const lastThrottle = new Map<string, number>()

export function getLogBuffer(): LogEntry[] {
  return LOG_BUFFER
}

export function clearLogBuffer() {
  LOG_BUFFER.length = 0
}

export function downloadLogBuffer(filename = 'kemudi-debug.json') {
  if (typeof document === 'undefined') return
  const blob = new Blob([JSON.stringify(LOG_BUFFER, null, 2)], { type: 'application/json' })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = filename
  a.click()
  URL.revokeObjectURL(url)
}
