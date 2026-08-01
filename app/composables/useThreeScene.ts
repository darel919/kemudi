import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'

export interface RendererConfig {
  pixelRatioCap: number
  antialias: boolean
  shadows: boolean
  toneMappingExposure: number
  shadowMapSize?: number
  /** Maximum physical drawing-buffer pixels before DPR is reduced. */
  renderPixelBudget?: number
}

export type SceneFrameCallback = (nowMs: number, dtMs: number) => void

export interface ThreeSceneHandle {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  renderer: THREE.WebGLRenderer
  controls: OrbitControls
  startLoop(onFrame: (dtMs: number) => void): void
  dispose(): void
}

function logDebug(event: string, data?: Record<string, unknown>) {
  if (import.meta.env.DEV) {
    console.debug(`[renderer:${event}]`, data ?? {})
  }
}

export function useThreeScene(canvas: HTMLCanvasElement, config: RendererConfig): ThreeSceneHandle {
  let disposedLocal = false
  let animFrame = 0
  let loopStarted = false
  let resizeQueued = false
  let sampleAccum = 0
  let sampleCount = 0
  let renderSampleAccum = 0
  let slowWindowCount = 0
  let lastSampleTime = performance.now()

  const width = Math.max(1, canvas.clientWidth || canvas.width || 1)
  const height = Math.max(1, canvas.clientHeight || canvas.height || 1)

  const scene = new THREE.Scene()
  const frameCallbacks = new Set<SceneFrameCallback>()
  scene.userData.kemudiFrameCallbacks = frameCallbacks
  scene.background = new THREE.Color(0x202028)
  scene.fog = new THREE.Fog(0x202028, 120, 220)

  const camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 500)
  camera.position.set(6, 5, 10)

  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: config.antialias,
    alpha: false,
    powerPreference: 'high-performance',
  })
  const configuredPixelRatio = Math.min(window.devicePixelRatio || 1, config.pixelRatioCap)
  const pixelBudget = config.renderPixelBudget ?? 4_500_000
  const getPixelRatio = (w: number, h: number) => Math.max(
    1,
    Math.min(configuredPixelRatio, Math.sqrt(pixelBudget / Math.max(1, w * h))),
  )
  let currentPixelRatio = getPixelRatio(width, height)
  renderer.setPixelRatio(currentPixelRatio)
  renderer.setSize(width, height, false)
  renderer.outputColorSpace = THREE.SRGBColorSpace
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = config.toneMappingExposure
  if (config.shadows) {
    renderer.shadowMap.enabled = true
    renderer.shadowMap.type = THREE.PCFShadowMap
  }

  const sun = new THREE.DirectionalLight(0xffffff, 2.2)
  sun.position.set(10, 18, 4)
  sun.castShadow = config.shadows
  if (config.shadows) {
    const shadowMapSize = config.shadowMapSize ?? 1024
    sun.shadow.mapSize.set(shadowMapSize, shadowMapSize)
    sun.shadow.camera.near = 1
    sun.shadow.camera.far = 50
    sun.shadow.camera.left = -20
    sun.shadow.camera.right = 20
    sun.shadow.camera.top = 20
    sun.shadow.camera.bottom = -20
    sun.shadow.bias = -0.001
  }
  scene.add(sun)

  const ambient = new THREE.AmbientLight(0x8899bb, 0.6)
  scene.add(ambient)

  const groundGeom = new THREE.PlaneGeometry(300, 300, 1, 1)
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x3a3a40,
    roughness: 0.95,
    metalness: 0.05,
    depthWrite: true,
  })
  const ground = new THREE.Mesh(groundGeom, groundMat)
  ground.userData.kemudiBaseGround = true
  ground.rotation.x = -Math.PI / 2
  ground.receiveShadow = config.shadows
  scene.add(ground)

  const controls = new OrbitControls(camera, canvas)
  controls.enableDamping = true
  controls.dampingFactor = 0.12
  controls.target.set(0, 0.6, 0)
  controls.update()

  const resizeObserver = new ResizeObserver((entries) => {
    if (resizeQueued || disposedLocal) return
    resizeQueued = true
    const entry = entries[0]
    if (!entry) {
      resizeQueued = false
      return
    }
    requestAnimationFrame(() => {
      if (disposedLocal) {
        resizeQueued = false
        return
      }
      const w = entry.contentRect.width
      const h = entry.contentRect.height
      if (Number.isFinite(w) && Number.isFinite(h) && w > 0 && h > 0) {
        currentPixelRatio = getPixelRatio(w, h)
        renderer.setPixelRatio(currentPixelRatio)
        renderer.setSize(Math.max(1, w), Math.max(1, h), false)
        camera.aspect = w / h
        camera.updateProjectionMatrix()
      }
      resizeQueued = false
    })
  })
  resizeObserver.observe(canvas)

  logDebug('initialized', {
    antialias: config.antialias,
    shadows: config.shadows,
    pixelRatio: currentPixelRatio,
    exposure: config.toneMappingExposure,
  })

  return {
    scene,
    camera,
    renderer,
    controls,

    startLoop(onFrame: (dtMs: number) => void) {
      if (loopStarted || disposedLocal) return
      loopStarted = true
      let prev = performance.now()

      const loop = () => {
        if (disposedLocal) return
        animFrame = requestAnimationFrame(loop)
        if (typeof document !== 'undefined' && document.hidden) return

        const now = performance.now()
        let dt = now - prev
        prev = now
        if (!Number.isFinite(dt) || dt <= 0) dt = 16.67

        if (controls.enabled) controls.update()
        const workStarted = performance.now()
        for (const callback of frameCallbacks) callback(now, dt)
        onFrame(dt)
        renderer.render(scene, camera)
        const frameWorkMs = performance.now() - workStarted

        sampleAccum += dt
        renderSampleAccum += frameWorkMs
        sampleCount += 1
        const sinceLast = now - lastSampleTime
        if (sinceLast >= 1000) {
          const avg = sampleAccum / sampleCount
          const avgFrameWorkMs = renderSampleAccum / sampleCount
          logDebug('frame-budget', {
            avgMs: +avg.toFixed(2),
            avgFrameWorkMs: +avgFrameWorkMs.toFixed(2),
            samples: sampleCount,
            windowMs: +sinceLast.toFixed(2),
            pixelRatio: currentPixelRatio,
          })
          if (avgFrameWorkMs > 32) slowWindowCount += 1
          else slowWindowCount = 0
          if (slowWindowCount >= 2 || (avgFrameWorkMs > 64 && slowWindowCount >= 1)) {
            if (currentPixelRatio > 1) {
              currentPixelRatio = Math.max(1, currentPixelRatio - 0.25)
              renderer.setPixelRatio(currentPixelRatio)
              logDebug('performance:budget-warning', {
                action: 'reduce-pixel-ratio',
                avgFrameWorkMs: +avgFrameWorkMs.toFixed(2),
                pixelRatio: currentPixelRatio,
              })
            } else if (renderer.shadowMap.enabled) {
              renderer.shadowMap.enabled = false
              sun.castShadow = false
              logDebug('performance:budget-warning', {
                action: 'disable-shadows',
                avgFrameWorkMs: +avgFrameWorkMs.toFixed(2),
              })
            }
            slowWindowCount = 0
          }
          sampleAccum = 0
          renderSampleAccum = 0
          sampleCount = 0
          lastSampleTime = now
        }
      }
      animFrame = requestAnimationFrame(loop)
    },

    dispose() {
      if (disposedLocal) return
      disposedLocal = true
      loopStarted = false
      if (animFrame) cancelAnimationFrame(animFrame)

      controls.dispose()
      frameCallbacks.clear()
      resizeObserver.disconnect()
      sun.dispose()
      ambient.dispose()
      groundGeom.dispose()
      groundMat.dispose()
      if (renderer) renderer.dispose()
    },
  }
}

// Alias for backward compatibility
export const createThreeScene = useThreeScene
