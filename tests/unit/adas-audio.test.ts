import { describe, expect, it, vi } from 'vitest'
import { createAdasAudioController, FCW_BEEP_ASSET, type AdasAudioElement } from '~/utils/adasAudio'

function fakeAudio(): AdasAudioElement & { play: ReturnType<typeof vi.fn>; pause: ReturnType<typeof vi.fn> } {
  return {
    currentTime: 0,
    loop: true,
    preload: 'auto',
    src: FCW_BEEP_ASSET,
    volume: 0.85,
    play: vi.fn(() => Promise.resolve()),
    pause: vi.fn(),
  }
}

describe('ADAS emergency audio', () => {
  it('plays the FCW beep once when AEB becomes active and loops it', () => {
    const audio = fakeAudio()
    const factory = vi.fn(() => audio)
    const controller = createAdasAudioController(factory)

    controller.setAebActive(false)
    expect(factory).not.toHaveBeenCalled()

    controller.setAebActive(true)
    controller.setAebActive(true)

    expect(factory).toHaveBeenCalledTimes(1)
    expect(audio.src).toBe('/assets/sounds/beep_generic_fcw.ogg')
    expect(audio.loop).toBe(true)
    expect(audio.play).toHaveBeenCalledTimes(1)
    expect(audio.currentTime).toBe(0)
  })

  it('stops and resets the beep when AEB ends, then allows a new intervention cue', () => {
    const audio = fakeAudio()
    const controller = createAdasAudioController(() => audio)

    controller.setAebActive(true)
    controller.setAebActive(false)

    expect(audio.pause).toHaveBeenCalledTimes(1)
    expect(audio.currentTime).toBe(0)

    controller.setAebActive(true)
    expect(audio.play).toHaveBeenCalledTimes(2)
  })

  it('cleans up the audio element on disposal', () => {
    const audio = fakeAudio()
    const controller = createAdasAudioController(() => audio)

    controller.setAebActive(true)
    controller.dispose()

    expect(audio.pause).toHaveBeenCalledTimes(1)
    expect(audio.src).toBe('')
    expect(audio.currentTime).toBe(0)
  })
})
