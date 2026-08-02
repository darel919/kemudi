export const FCW_BEEP_ASSET = '/assets/sounds/beep_generic_fcw.ogg'

export interface AdasAudioController {
  setAebActive(active: boolean): void
  dispose(): void
}

export interface AdasAudioElement {
  currentTime: number
  loop: boolean
  preload: string
  src: string
  volume: number
  pause(): void
  play(): Promise<void> | void
}

type AudioFactory = () => AdasAudioElement | null

function createBrowserAudio(): AdasAudioElement | null {
  if (typeof window === 'undefined' || typeof Audio === 'undefined') return null
  const audio = new Audio(FCW_BEEP_ASSET)
  audio.preload = 'auto'
  audio.loop = true
  audio.volume = 0.85
  return audio
}

/**
 * Drives the FCW beep from the authoritative AEB state.
 * The short FCW asset loops only while emergency braking is active, and
 * repeated telemetry snapshots do not restart the sound every tick.
 */
export function createAdasAudioController(factory: AudioFactory = createBrowserAudio): AdasAudioController {
  let audio: AdasAudioElement | null = null
  let aebActive = false

  return {
    setAebActive(active: boolean) {
      if (active === aebActive) return
      aebActive = active

      if (active) {
        audio ??= factory()
        if (!audio) return
        audio.src = FCW_BEEP_ASSET
        audio.currentTime = 0
        try {
          const playResult = audio.play()
          if (playResult && typeof playResult.catch === 'function') {
            void playResult.catch(() => undefined)
          }
        } catch {
          // Browser autoplay policy may reject playback synchronously.
        }
        return
      }

      audio?.pause()
      if (audio) audio.currentTime = 0
    },

    dispose() {
      aebActive = false
      audio?.pause()
      if (audio) {
        audio.currentTime = 0
        audio.src = ''
      }
      audio = null
    },
  }
}
