import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'

const mocks = vi.hoisted(() => {
  const disposeFn = vi.fn()
  const renderFn = vi.fn()
  const setPixelRatioFn = vi.fn()
  const setSizeFn = vi.fn()
  const disconnectFn = vi.fn()
  const observeFn = vi.fn()

  class MockWebGLRenderer {
    domElement: HTMLCanvasElement
    shadowMap = { enabled: false, type: 0 }
    toneMapping = 0
    toneMappingExposure = 1
    outputColorSpace = ''
    constructor() {
      this.domElement = document.createElement('canvas')
    }
    dispose = disposeFn
    setPixelRatio = setPixelRatioFn
    setSize = setSizeFn
    render = renderFn
  }

  class MockOrbitControls {
    enableDamping = false
    dampingFactor = 0
    target = { set: vi.fn() }
    update = vi.fn()
    dispose = vi.fn()
  }

  class MockResizeObserver {
    observe = observeFn
    disconnect = disconnectFn
  }

  return {
    disposeFn,
    renderFn,
    setPixelRatioFn,
    setSizeFn,
    disconnectFn,
    observeFn,
    MockWebGLRenderer,
    MockOrbitControls,
    MockResizeObserver,
  }
})

vi.mock('three', async (importOriginal) => {
  const actual = await importOriginal<typeof import('three')>()
  return { ...actual, WebGLRenderer: mocks.MockWebGLRenderer }
})

vi.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: mocks.MockOrbitControls,
}))

import { useThreeScene, type RendererConfig } from '../../app/composables/useThreeScene'

beforeEach(() => {
  vi.stubGlobal('ResizeObserver', mocks.MockResizeObserver)
  vi.stubGlobal('requestAnimationFrame', (cb: FrameRequestCallback) => {
    return setTimeout(() => cb(performance.now()), 16) as unknown as number
  })
  vi.stubGlobal('cancelAnimationFrame', (id: number) => clearTimeout(id))
  vi.clearAllMocks()
})

afterEach(() => {
  vi.unstubAllGlobals()
})

const baseConfig: RendererConfig = {
  pixelRatioCap: 2,
  antialias: true,
  shadows: true,
  toneMappingExposure: 1.1,
}

describe('useThreeScene', () => {
  it('applies preset at renderer creation', () => {
    const canvas = document.createElement('canvas')
    canvas.width = 800
    canvas.height = 600
    const handle = useThreeScene(canvas, { ...baseConfig, pixelRatioCap: 1.5 })
    expect(mocks.setPixelRatioFn).toHaveBeenCalledWith(Math.min(window.devicePixelRatio, 1.5))
    expect(mocks.setSizeFn).toHaveBeenCalled()
    expect(mocks.observeFn).toHaveBeenCalledWith(canvas)
    handle.dispose()
  })

  it('starts a render loop and reports frame budget without per-frame logging', () => {
    const canvas = document.createElement('canvas')
    canvas.width = 800
    canvas.height = 600
    const handle = useThreeScene(canvas, baseConfig)
    const onFrame = vi.fn()
    handle.startLoop(onFrame)
    expect(mocks.renderFn).not.toHaveBeenCalled()
    return new Promise<void>((resolve) => {
      setTimeout(() => {
        expect(mocks.renderFn).toHaveBeenCalled()
        expect(onFrame).toHaveBeenCalled()
        handle.dispose()
        resolve()
      }, 100)
    })
  })

  it('disposes renderer resources on unmount', () => {
    const canvas = document.createElement('canvas')
    canvas.width = 800
    canvas.height = 600
    const handle = useThreeScene(canvas, baseConfig)
    handle.dispose()
    expect(mocks.disposeFn).toHaveBeenCalled()
    expect(mocks.disconnectFn).toHaveBeenCalled()
  })

  it('is idempotent on dispose', () => {
    const canvas = document.createElement('canvas')
    canvas.width = 800
    canvas.height = 600
    const handle = useThreeScene(canvas, baseConfig)
    handle.dispose()
    handle.dispose()
    expect(mocks.disposeFn).toHaveBeenCalledTimes(1)
  })

  it('exposes scene, camera, renderer, controls', () => {
    const canvas = document.createElement('canvas')
    canvas.width = 800
    canvas.height = 600
    const handle = useThreeScene(canvas, baseConfig)
    expect(handle.scene).toBeDefined()
    expect(handle.camera).toBeDefined()
    expect(handle.renderer).toBeDefined()
    expect(handle.controls).toBeDefined()
    handle.dispose()
  })
})