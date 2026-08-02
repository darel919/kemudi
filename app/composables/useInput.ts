import { ref, readonly } from 'vue'
import { logDebug } from '~/utils/debug'

export interface InputState {
  steering: number    // -1 (left) to +1 (right)
  throttle: number    // 0 to 1
  brake: number       // 0 to 1
  clutch: number      // 0 to 1
  handbrake: boolean
  gearUp: boolean
  gearDown: boolean
  ignitionToggle: boolean
  transmissionToggle: boolean // toggle manual/auto
}

const DEFAULT_KEYBINDS: Record<string, keyof InputState> = {
  ArrowLeft: 'steering',
  ArrowRight: 'steering',
  ArrowUp: 'throttle',
  ArrowDown: 'brake',
  KeyW: 'throttle',
  KeyS: 'brake',
  KeyA: 'steering',
  KeyD: 'steering',
  ShiftLeft: 'clutch',
  Space: 'handbrake',
  KeyE: 'ignitionToggle',
  KeyR: 'gearUp',
  KeyQ: 'gearDown',
  KeyT: 'transmissionToggle',
}

export function normalizeGamepadInput(value: number | undefined, deadzone = 0.05): number {
  if (!Number.isFinite(value)) return 0
  const clamped = Math.max(0, Math.min(1, value!))
  return clamped > deadzone ? clamped : 0
}

export function useInput() {
  const state = ref<InputState>({
    steering: 0,
    throttle: 0,
    brake: 0,
    clutch: 0,
    handbrake: false,
    gearUp: false,
    gearDown: false,
    ignitionToggle: false,
    transmissionToggle: false,
  })

  let disposed = false
  const keysDown = new Set<string>()
  let gamepadIndex: number | null = null

  function onKeyDown(e: KeyboardEvent) {
    if (disposed) return
    keysDown.add(e.code)
    updateFromKeys()
  }

  function onKeyUp(e: KeyboardEvent) {
    if (disposed) return
    keysDown.delete(e.code)
    updateFromKeys()
  }

  function onBlur() {
    if (disposed) return
    keysDown.clear()
    resetState()
  }

  function onVisibilityChange() {
    if (typeof document !== 'undefined' && document.visibilityState === 'hidden') onBlur()
  }

  function onGamepadConnected(e: GamepadEvent) {
    if (gamepadIndex === null) gamepadIndex = e.gamepad.index
  }

  function onGamepadDisconnected(e: GamepadEvent) {
    if (gamepadIndex === e.gamepad.index) {
      gamepadIndex = null
      resetState()
    }
  }

  function updateFromKeys() {
    const s = state.value
    s.steering = 0
    s.throttle = 0
    s.brake = 0
    s.clutch = 0
    s.handbrake = false
    s.gearUp = false
    s.gearDown = false
    s.ignitionToggle = false
    s.transmissionToggle = false

    for (const code of keysDown) {
      const action = DEFAULT_KEYBINDS[code]
      if (!action) continue

      switch (action) {
        case 'steering':
          if (code === 'ArrowLeft' || code === 'KeyA') s.steering -= 1
          if (code === 'ArrowRight' || code === 'KeyD') s.steering += 1
          break
        case 'throttle':
          s.throttle = 1
          break
        case 'brake':
          s.brake = 1
          break
        case 'clutch':
          s.clutch = 1
          break
        case 'handbrake':
          s.handbrake = true
          break
        case 'gearUp':
          s.gearUp = true
          break
        case 'gearDown':
          s.gearDown = true
          break
        case 'ignitionToggle':
          s.ignitionToggle = true
          break
        case 'transmissionToggle':
          s.transmissionToggle = true
          break
      }
    }

    // Clamp steering
    s.steering = Math.max(-1, Math.min(1, s.steering))
  }

  function resetState() {
    state.value = {
      steering: 0, throttle: 0, brake: 0, clutch: 0,
      handbrake: false, gearUp: false, gearDown: false, ignitionToggle: false, transmissionToggle: false,
    }
  }

  function init() {
    if (typeof window === 'undefined') return
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', onBlur)
    document.addEventListener('visibilitychange', onVisibilityChange)
    window.addEventListener('gamepadconnected', onGamepadConnected)
    window.addEventListener('gamepaddisconnected', onGamepadDisconnected)
    logDebug('renderer:initialized', { component: 'InputSystem' })
  }

  /** Poll the selected gamepad and merge its normalized controls into input state. */
  function pollGamepad() {
    if (disposed || typeof navigator === 'undefined' || !navigator.getGamepads) return
    const pads = navigator.getGamepads()
    const pad = gamepadIndex === null
      ? Array.from(pads).find((candidate): candidate is Gamepad => candidate !== null)
      : pads[gamepadIndex]
    if (!pad) return
    if (gamepadIndex === null) gamepadIndex = pad.index

    updateFromKeys()
    const axis = (pad.axes[0] ?? 0)
    if (Math.abs(axis) > 0.05) state.value.steering = Math.max(-1, Math.min(1, axis))
    state.value.throttle = Math.max(state.value.throttle, normalizeGamepadInput(pad.buttons[7]?.value))
    state.value.brake = Math.max(state.value.brake, normalizeGamepadInput(pad.buttons[6]?.value))
    state.value.handbrake = state.value.handbrake || !!pad.buttons[0]?.pressed
    state.value.gearUp = state.value.gearUp || !!pad.buttons[5]?.pressed
    state.value.gearDown = state.value.gearDown || !!pad.buttons[4]?.pressed
  }

  function dispose() {
    if (disposed) return
    disposed = true
    if (typeof window !== 'undefined') {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      window.removeEventListener('blur', onBlur)
      document.removeEventListener('visibilitychange', onVisibilityChange)
      window.removeEventListener('gamepadconnected', onGamepadConnected)
      window.removeEventListener('gamepaddisconnected', onGamepadDisconnected)
    }
    gamepadIndex = null
    resetState()
  }

  return {
    state: readonly(state),
    init,
    pollGamepad,
    dispose,
  }
}
