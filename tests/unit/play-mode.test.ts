import { describe, it, expect, beforeEach, vi } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'

vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

import {
  PLAY_MODE_DEFINITIONS,
  DRAG_RACE_CONFIG,
  DRAG_STRIP_SPLITS,
  type PlayModeId,
} from '../../app/types/play-mode'
import { usePlayModeStore } from '../../app/stores/playMode'
import { usePlayMode } from '../../app/composables/usePlayMode'

describe('PlayMode types & definitions', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('has definitions for freeroam and drag-race', () => {
    expect(PLAY_MODE_DEFINITIONS.freeroam).toBeDefined()
    expect(PLAY_MODE_DEFINITIONS['drag-race']).toBeDefined()
  })

  it('freeroam has no timer and no finish condition', () => {
    const def = PLAY_MODE_DEFINITIONS.freeroam
    expect(def.rules.hasTimer).toBe(false)
    expect(def.rules.hasFinishCondition).toBe(false)
  })

  it('drag-race has timer and finish condition', () => {
    const def = PLAY_MODE_DEFINITIONS['drag-race']
    expect(def.rules.hasTimer).toBe(true)
    expect(def.rules.hasFinishCondition).toBe(true)
  })

  it('drag-race track length is exactly 1609.344', () => {
    expect(DRAG_RACE_CONFIG.trackLengthMeters).toBe(1609.344)
  })

  it('drag-race splits at correct distances', () => {
    expect(DRAG_STRIP_SPLITS).toEqual([18.288, 100.584, 402.336, 804.672, 1609.344])
  })

  it('drag-race definition includes dragRace config', () => {
    const def = PLAY_MODE_DEFINITIONS['drag-race']
    expect(def.dragRace).toBeDefined()
    expect(def.dragRace!.trackLengthMeters).toBe(1609.344)
    expect(def.dragRace!.splits).toHaveLength(5)
  })

  it('freeroam definition has no dragRace config', () => {
    expect(PLAY_MODE_DEFINITIONS.freeroam.dragRace).toBeUndefined()
  })
})

describe('PlayMode store', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('starts in idle lifecycle with freeroam mode', () => {
    const store = usePlayModeStore()
    expect(store.lifecycle).toBe('idle')
    expect(store.mode).toBe('freeroam')
    expect(store.isRunning).toBe(false)
    expect(store.dragRacePhase).toBe('none')
    expect(store.dragRaceResult).toBeNull()
  })

  it('setMode sets mode, isRunning, and lifecycle', () => {
    const store = usePlayModeStore()
    store.setMode('drag-race')
    expect(store.mode).toBe('drag-race')
    expect(store.isRunning).toBe(true)
    expect(store.lifecycle).toBe('active')
  })

  it('reset returns to initial state', () => {
    const store = usePlayModeStore()
    store.setMode('drag-race')
    store.setDragRacePhase('running')
    store.reset()
    expect(store.mode).toBe('freeroam')
    expect(store.isRunning).toBe(false)
    expect(store.lifecycle).toBe('idle')
    expect(store.dragRacePhase).toBe('none')
    expect(store.dragRaceResult).toBeNull()
  })

  it('canExit is false during loading', () => {
    const store = usePlayModeStore()
    store.setLifecycle('loading')
    expect(store.canExit).toBe(false)
  })

  it('canExit is true when active', () => {
    const store = usePlayModeStore()
    store.setLifecycle('active')
    expect(store.canExit).toBe(true)
  })
})

describe('usePlayMode lifecycle', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('getModeDefinition returns correct definition', () => {
    const pm = usePlayMode()
    const def = pm.getModeDefinition('drag-race')
    expect(def.id).toBe('drag-race')
    expect(def.label).toBe('Drag Race')
  })

  it('enterMode sets lifecycle: loading → active', () => {
    const pm = usePlayMode()
    expect(pm.getCurrentPhase()).toBe('idle')

    pm.enterMode('drag-race')
    expect(pm.getCurrentPhase()).toBe('active')
    expect(pm.store.mode).toBe('drag-race')
  })

  it('exitMode cleans up: active → ended → idle', () => {
    const pm = usePlayMode()
    pm.enterMode('freeroam')
    expect(pm.getCurrentPhase()).toBe('active')

    pm.exitMode()
    expect(pm.getCurrentPhase()).toBe('idle')
    expect(pm.store.mode).toBe('freeroam')
    expect(pm.store.isRunning).toBe(false)
  })

  it('pauseMode transitions active → paused', () => {
    const pm = usePlayMode()
    pm.enterMode('freeroam')
    pm.pauseMode()
    expect(pm.getCurrentPhase()).toBe('paused')
  })

  it('resumeMode transitions paused → active', () => {
    const pm = usePlayMode()
    pm.enterMode('freeroam')
    pm.pauseMode()
    pm.resumeMode()
    expect(pm.getCurrentPhase()).toBe('active')
  })

  it('full lifecycle: idle → loading → active → paused → active → ended → idle', () => {
    const pm = usePlayMode()
    expect(pm.getCurrentPhase()).toBe('idle')

    pm.enterMode('freeroam')
    expect(pm.getCurrentPhase()).toBe('active')

    pm.pauseMode()
    expect(pm.getCurrentPhase()).toBe('paused')

    pm.resumeMode()
    expect(pm.getCurrentPhase()).toBe('active')

    pm.exitMode()
    expect(pm.getCurrentPhase()).toBe('idle')
  })

  it('resetMode resets without exiting', () => {
    const pm = usePlayMode()
    pm.enterMode('drag-race')
    pm.store.setDragRacePhase('running')

    pm.resetMode()
    expect(pm.store.mode).toBe('freeroam')
    expect(pm.store.dragRacePhase).toBe('none')
    expect(pm.store.lifecycle).toBe('idle')
  })

  it('cannot pause from idle', () => {
    const pm = usePlayMode()
    pm.pauseMode()
    expect(pm.getCurrentPhase()).toBe('idle')
  })

  it('cannot resume from idle', () => {
    const pm = usePlayMode()
    pm.resumeMode()
    expect(pm.getCurrentPhase()).toBe('idle')
  })

  it('cannot exit from idle', () => {
    const pm = usePlayMode()
    pm.exitMode()
    expect(pm.getCurrentPhase()).toBe('idle')
  })
})

describe('usePlayMode drag race', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('getDragRacePhase starts at none', () => {
    const pm = usePlayMode()
    expect(pm.getDragRacePhase()).toBe('none')
  })

  it('advanceDragRacePhase progresses: none → staging → pre-stage → ... → finished', () => {
    const pm = usePlayMode()
    const expected: string[] = [
      'staging', 'pre-stage', 'stage', 'countdown',
      'launch', 'running', 'finished',
    ]
    for (const phase of expected) {
      const ok = pm.advanceDragRacePhase()
      expect(ok).toBe(true)
      expect(pm.getDragRacePhase()).toBe(phase)
    }
  })

  it('setDragRacePhase follows valid transitions', () => {
    const pm = usePlayMode()
    pm.store.setDragRacePhase('countdown')

    const ok = pm.setDragRacePhase('launch')
    expect(ok).toBe(true)
    expect(pm.getDragRacePhase()).toBe('launch')
  })

  it('setDragRacePhase rejects invalid transitions', () => {
    const pm = usePlayMode()
    // none → launch is not valid
    const ok = pm.setDragRacePhase('launch')
    expect(ok).toBe(false)
    expect(pm.getDragRacePhase()).toBe('none')
  })

  it('getTrackLength returns 1609.344', () => {
    const pm = usePlayMode()
    expect(pm.getTrackLength()).toBe(1609.344)
  })

  it('getSplits returns 5 splits with correct distances', () => {
    const pm = usePlayMode()
    const splits = pm.getSplits(10.0)
    expect(splits).toHaveLength(5)
    expect(splits[0].distance).toBe(18.288)
    expect(splits[4].distance).toBe(1609.344)
    // Last split time should equal elapsed time
    expect(splits[4].time).toBeCloseTo(10.0, 10)
  })

  it('interpolates split times from authoritative distance samples', () => {
    const pm = usePlayMode()
    const splits = pm.getSplits(10.0, [
      { distance: 0, time: 0, speed: 0 },
      { distance: 402.336, time: 5, speed: 40 },
      { distance: 1609.344, time: 12, speed: 50 },
    ])
    expect(splits[2].time).toBeCloseTo(5, 10)
    expect(splits[2].speed).toBeCloseTo(40, 10)
    expect(splits[4].time).toBeCloseTo(12, 10)
  })

  it('completeDragRace stores result and sets phase to finished', () => {
    const pm = usePlayMode()
    pm.store.setDragRacePhase('running')

    const result = {
      reactionTime: 0.15,
      elapsedTime: 12.5,
      trapSpeed: 45.2,
      peakSpeed: 48.1,
      splits: [],
      finishReason: 'completed' as const,
      lane: 1,
      gearShifts: 3,
      maxWheelSlip: 0.15,
    }
    pm.completeDragRace(result)
    expect(pm.getDragRacePhase()).toBe('finished')
    expect(pm.store.dragRaceResult).toEqual(result)
  })

  it('detectFalseStart detects movement during countdown', () => {
    const pm = usePlayMode()
    pm.store.setDragRacePhase('countdown')

    const detected = pm.detectFalseStart(true)
    expect(detected).toBe(true)
    expect(pm.getDragRacePhase()).toBe('false-start')
  })

  it('detectFalseStart ignores movement after launch', () => {
    const pm = usePlayMode()
    pm.store.setDragRacePhase('launch')

    const detected = pm.detectFalseStart(true)
    expect(detected).toBe(false)
    expect(pm.getDragRacePhase()).toBe('launch')
  })

  it('detectFalseStart ignores no-movement during countdown', () => {
    const pm = usePlayMode()
    pm.store.setDragRacePhase('countdown')

    const detected = pm.detectFalseStart(false)
    expect(detected).toBe(false)
    expect(pm.getDragRacePhase()).toBe('countdown')
  })

  it('canStartDragRace rejects if not in drag-race mode', () => {
    const pm = usePlayMode()
    pm.enterMode('freeroam')
    const result = pm.canStartDragRace(1.0, 0)
    expect(result.ok).toBe(false)
    expect(result.reason).toBe('not in drag-race mode')
  })

  it('canStartDragRace rejects empty fuel', () => {
    const pm = usePlayMode()
    pm.enterMode('drag-race')
    const result = pm.canStartDragRace(0, 0)
    expect(result.ok).toBe(false)
    expect(result.reason).toBe('empty fuel')
  })

  it('canStartDragRace rejects destroyed vehicle', () => {
    const pm = usePlayMode()
    pm.enterMode('drag-race')
    const result = pm.canStartDragRace(1.0, 1.0)
    expect(result.ok).toBe(false)
    expect(result.reason).toBe('vehicle destroyed')
  })

  it('canStartDragRace accepts valid vehicle', () => {
    const pm = usePlayMode()
    pm.enterMode('drag-race')
    const result = pm.canStartDragRace(1.0, 0.3)
    expect(result.ok).toBe(true)
  })

  it('exitMode resets drag race phase to none', () => {
    const pm = usePlayMode()
    pm.enterMode('drag-race')
    pm.store.setDragRacePhase('running')
    pm.exitMode()
    expect(pm.getDragRacePhase()).toBe('none')
  })
})

describe('DragRaceResult structure', () => {
  it('has all required fields', () => {
    const result: import('../../app/types/play-mode').DragRaceResult = {
      reactionTime: 0.15,
      elapsedTime: 12.5,
      trapSpeed: 45.2,
      peakSpeed: 48.1,
      splits: [
        { distance: 18.288, time: 1.5, speed: 12.1 },
        { distance: 100.584, time: 4.2, speed: 23.9 },
        { distance: 402.336, time: 8.0, speed: 40.0 },
        { distance: 804.672, time: 10.5, speed: 44.2 },
        { distance: 1609.344, time: 12.5, speed: 45.2 },
      ],
      finishReason: 'completed',
      lane: 1,
      gearShifts: 3,
      maxWheelSlip: 0.15,
    }

    expect(result.reactionTime).toBeTypeOf('number')
    expect(result.elapsedTime).toBeTypeOf('number')
    expect(result.trapSpeed).toBeTypeOf('number')
    expect(result.peakSpeed).toBeTypeOf('number')
    expect(result.splits).toBeInstanceOf(Array)
    expect(result.splits).toHaveLength(5)
    expect(result.finishReason).toBeOneOf([
      'completed', 'timeout', 'false-start', 'disqualified', 'abandoned',
    ])
    expect(result.lane).toBeTypeOf('number')
    expect(result.gearShifts).toBeTypeOf('number')
    expect(result.maxWheelSlip).toBeTypeOf('number')
  })
})
