import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'

vi.mock('~/utils/debug', () => ({
  logDebug: vi.fn(),
}))

import { normalizeGamepadInput, useInput } from '../../app/composables/useInput'

describe('useInput', () => {
  let input: ReturnType<typeof useInput>

  beforeEach(() => {
    input = useInput()
    vi.stubGlobal('window', {
      addEventListener: vi.fn(),
      removeEventListener: vi.fn(),
    })
    vi.stubGlobal('KeyboardEvent', class {
      code: string
      constructor(type: string, opts: { code?: string } = {}) {
        this.code = opts.code ?? ''
      }
    })
  })

  afterEach(() => {
    input.dispose()
    vi.unstubAllGlobals()
  })

  it('initializes with zero state', () => {
    const s = input.state.value
    expect(s.steering).toBe(0)
    expect(s.throttle).toBe(0)
    expect(s.brake).toBe(0)
    expect(s.clutch).toBe(0)
    expect(s.handbrake).toBe(false)
  })

  it('rejects invalid trigger values and applies a dead zone', () => {
    expect(normalizeGamepadInput(undefined)).toBe(0)
    expect(normalizeGamepadInput(Number.NaN)).toBe(0)
    expect(normalizeGamepadInput(0.04)).toBe(0)
    expect(normalizeGamepadInput(0.25)).toBe(0.25)
    expect(normalizeGamepadInput(2)).toBe(1)
  })

  it('init registers event listeners', () => {
    input.init()
    expect(window.addEventListener).toHaveBeenCalled()
  })

  it('dispose removes event listeners', () => {
    input.init()
    input.dispose()
    expect(window.removeEventListener).toHaveBeenCalled()
  })

  it('dispose is idempotent', () => {
    input.init()
    input.dispose()
    input.dispose() // should not throw
  })

  it('state is readonly', () => {
    const s = input.state
    // @ts-expect-error testing readonly
    s.value = { ...s.value, throttle: 999 }
    expect(input.state.value.throttle).toBe(0)
  })
})
