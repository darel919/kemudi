import {
  usePlayModeStore,
  type PlayModeConfig,
} from '~/stores/playMode'
import {
  PLAY_MODE_DEFINITIONS,
  DRAG_RACE_CONFIG,
  DRAG_RACE_TRANSITIONS,
  type PlayModeId,
  type PlayModeDefinition,
  type PlayModeLifecycle,
  type DragRacePhase,
  type DragRaceResult,
} from '~/types/play-mode'
import { logDebug } from '~/utils/debug'

/** Lifecycle state machine — valid next-states from a given lifecycle */
const LIFECYCLE_TRANSITIONS: Record<PlayModeLifecycle, Set<PlayModeLifecycle>> = {
  idle:    new Set(['loading']),
  loading: new Set(['active', 'error']),
  active:  new Set(['paused', 'ended']),
  paused:  new Set(['active', 'ended']),
  ended:   new Set(['idle']),
  error:   new Set(['idle']),
}

export function usePlayMode() {
  const store = usePlayModeStore()

  // ── Mode definition ──────────────────────────────────────────────

  function getModeDefinition(id: PlayModeId): PlayModeDefinition {
    return PLAY_MODE_DEFINITIONS[id]
  }

  // ── Lifecycle ────────────────────────────────────────────────────

  function canTransition(to: PlayModeLifecycle): boolean {
    return LIFECYCLE_TRANSITIONS[store.lifecycle]?.has(to) ?? false
  }

  function enterMode(id: PlayModeId) {
    if (!canTransition('loading')) {
      logDebug('play-mode:lifecycle', { error: 'cannot enter', from: store.lifecycle })
      return
    }
    store.setLifecycle('loading')
    logDebug('play-mode:lifecycle', { event: 'enter', mode: id })

    // Loading completes immediately (no async asset loading yet — Phase 6.2+)
    if (canTransition('active')) {
      store.setMode(id)
      logDebug('play-mode:lifecycle', { event: 'active', mode: id })
    }
  }

  function exitMode() {
    if (!canTransition('ended')) {
      logDebug('play-mode:lifecycle', { error: 'cannot exit', from: store.lifecycle })
      return
    }
    store.setLifecycle('ended')
    logDebug('play-mode:lifecycle', { event: 'exit' })

    // Ended → idle in one tick (cleanup is synchronous here)
    if (canTransition('idle')) {
      store.reset()
      logDebug('play-mode:lifecycle', { event: 'idle' })
    }
  }

  function pauseMode() {
    if (!canTransition('paused')) {
      logDebug('play-mode:lifecycle', { error: 'cannot pause', from: store.lifecycle })
      return
    }
    store.setLifecycle('paused')
    logDebug('play-mode:lifecycle', { event: 'pause' })
  }

  function resumeMode() {
    if (!canTransition('active')) {
      logDebug('play-mode:lifecycle', { error: 'cannot resume', from: store.lifecycle })
      return
    }
    store.setLifecycle('active')
    logDebug('play-mode:lifecycle', { event: 'resume' })
  }

  function resetMode() {
    const prevMode = store.mode
    store.reset()
    logDebug('play-mode:lifecycle', { event: 'reset', previousMode: prevMode })
  }

  function getCurrentPhase(): PlayModeLifecycle {
    return store.lifecycle
  }

  // ── Drag race ────────────────────────────────────────────────────

  function getDragRacePhase(): DragRacePhase {
    return store.dragRacePhase
  }

  function advanceDragRacePhase(): boolean {
    const current = store.dragRacePhase
    const nexts = DRAG_RACE_TRANSITIONS[current]
    if (!nexts || nexts.size === 0) {
      logDebug('play-mode:drag-race-phase', { error: 'terminal phase', phase: current })
      return false
    }
    // Advance to the first valid successor (ordered by typical flow)
    const ORDER: DragRacePhase[] = [
      'staging', 'pre-stage', 'stage', 'countdown',
      'launch', 'running', 'finished',
    ]
    const next = ORDER.find(p => nexts.has(p)) ?? nexts.values().next().value!
    store.setDragRacePhase(next)
    logDebug('play-mode:drag-race-phase', { from: current, to: next })
    return true
  }

  function setDragRacePhase(phase: DragRacePhase): boolean {
    const current = store.dragRacePhase
    if (current === phase) return true
    const valid = DRAG_RACE_TRANSITIONS[current]?.has(phase)
    if (!valid) {
      logDebug('play-mode:drag-race-phase', { error: 'invalid transition', from: current, to: phase })
      return false
    }
    store.setDragRacePhase(phase)
    logDebug('play-mode:drag-race-phase', { from: current, to: phase })
    return true
  }

  function getTrackLength(): number {
    return DRAG_RACE_CONFIG.trackLengthMeters
  }

  function getSplits(
    elapsedTime: number,
    samples: readonly { distance: number; time: number; speed?: number }[] = [],
  ): { distance: number; time: number; speed?: number }[] {
    const total = DRAG_RACE_CONFIG.trackLengthMeters
    const splits = DRAG_RACE_CONFIG.splits
    const ordered = samples
      .filter(sample => Number.isFinite(sample.distance) && Number.isFinite(sample.time))
      .sort((a, b) => a.distance - b.distance)

    return splits.map(distance => {
      const after = ordered.find(sample => sample.distance >= distance)
      const beforeIndex = after ? ordered.indexOf(after) - 1 : ordered.length - 1
      const before = beforeIndex >= 0 ? ordered[beforeIndex] : undefined
      if (before && after && after.distance > before.distance) {
        const amount = (distance - before.distance) / (after.distance - before.distance)
        const result: { distance: number; time: number; speed?: number } = {
          distance,
          time: before.time + (after.time - before.time) * amount,
        }
        if (before.speed !== undefined && after.speed !== undefined) {
          result.speed = before.speed + (after.speed - before.speed) * amount
        }
        return result
      }
      return { distance, time: (distance / total) * elapsedTime }
    })
  }

  function completeDragRace(result: DragRaceResult): boolean {
    const phase: DragRacePhase = result.finishReason === 'completed'
      ? 'finished'
      : result.finishReason === 'timeout'
        ? 'timeout'
        : result.finishReason === 'abandoned'
          ? 'abandoned'
          : result.finishReason === 'false-start'
            ? 'false-start'
            : result.finishReason === 'disqualified'
              ? 'disqualified'
              : 'none'
    if (!DRAG_RACE_TRANSITIONS[store.dragRacePhase]?.has(phase)) return false
    store.setDragRaceResult(result)
    store.setDragRacePhase(phase)
    logDebug('play-mode:drag-race-result', { finishReason: result.finishReason })
    return true
  }

  // ── Validation helpers ───────────────────────────────────────────

  function canStartDragRace(vehicleFuelLevel: number, vehicleDamage: number): { ok: boolean; reason?: string } {
    if (store.mode !== 'drag-race') return { ok: false, reason: 'not in drag-race mode' }
    if (vehicleFuelLevel <= 0) return { ok: false, reason: 'empty fuel' }
    if (vehicleDamage >= 1) return { ok: false, reason: 'vehicle destroyed' }
    return { ok: true }
  }

  function detectFalseStart(
    movementBeforeGo: boolean,
  ): boolean {
    if (store.dragRacePhase === 'countdown' && movementBeforeGo) {
      store.setDragRacePhase('false-start')
      logDebug('play-mode:drag-race-phase', { event: 'false-start' })
      return true
    }
    return false
  }

  // ── Public API ───────────────────────────────────────────────────

  return {
    // Definitions
    getModeDefinition,

    // Lifecycle
    enterMode,
    exitMode,
    pauseMode,
    resumeMode,
    resetMode,
    getCurrentPhase,

    // Drag race
    getDragRacePhase,
    advanceDragRacePhase,
    setDragRacePhase,
    getTrackLength,
    getSplits,
    completeDragRace,
    canStartDragRace,
    detectFalseStart,

    // Expose store for read access
    store,
  }
}

// Re-export types for consumer convenience
export type {
  PlayModeDefinition,
  DragRaceResult,
}
