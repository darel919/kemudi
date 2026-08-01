import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'

export interface RendererConfig {
  pixelRatioCap: number
  antialias: boolean
  shadows: boolean
  toneMappingExposure: number
}

export interface ThreeSceneHandle {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  renderer: THREE.WebGLRenderer
  controls: OrbitControls
  startLoop(onFrame: (dtMs: number) => void): void
  dispose(): void
}

let sampleAccum = 0
let sampleCount = 0
let lastSampleTime = performance.now()

function logDebug(event: string, data?: Record<string, unknown>) {
  if (import.meta.env.DEV) {
    console.debug(`[renderer:${event}]`, data ?? {})
  }
}

export function useThreeScene(canvas: HTMLCanvasElement, config: RendererConfig): ThreeSceneHandle {
  let disposedLocal = false
  let animFrame = 0
  let resizeQueued = false

  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0x202028)
  scene.fog = new THREE.Fog(0x202028, 120, 220)

  const camera = new THREE.PerspectiveCamera(60, canvas.clientWidth / canvas.clientHeight, 0.1, 500)
  camera.position.set(6, 5, 10)

  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: config.antialias,
    alpha: false,
    powerPreference: 'high-performance',
  })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, config.pixelRatioCap))
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
  renderer.outputColorSpace = THREE.SRGBColorSpace
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = config.toneMappingExposure
  if (config.shadows) {
    renderer.shadowMap.enabled = true
    renderer.shadowMap.type = THREE.PCFSoftShadowMap
  }

  const sun = new THREE.DirectionalLight(0xffffff, 2.2)
  sun.position.set(10, 18, 4)
  sun.castShadow = config.shadows
  if (config.shadows) {
    sun.shadow.mapSize.set(1024, 1024)
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
        renderer.setSize(w, h, false)
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
    pixelRatio: Math.min(window.devicePixelRatio, config.pixelRatioCap),
    exposure: config.toneMappingExposure,
  })

  return {
    scene,
    camera,
    renderer,
    controls,

    startLoop(onFrame: (dtMs: number) => void) {
      let prev = performance.now()

      const loop = () => {
        if (disposedLocal) return
        animFrame = requestAnimationFrame(loop)

        const now = performance.now()
        let dt = now - prev
        prev = now
        if (!Number.isFinite(dt) || dt <= 0) dt = 16.67

        controls.update()
        renderer.render(scene, camera)
        onFrame(dt)

        sampleAccum += dt
        sampleCount += 1
        const sinceLast = now - lastSampleTime
        if (sinceLast >= 1000) {
          const avg = sampleAccum / sampleCount
          logDebug('frame-budget', {
            avgMs: +avg.toFixed(2),
            samples: sampleCount,
            windowMs: +sinceLast.toFixed(2),
          })
          sampleAccum = 0
          sampleCount = 0
          lastSampleTime = now
        }
      }
      animFrame = requestAnimationFrame(loop)
    },

    dispose() {
      if (disposedLocal) return
      disposedLocal = true

      controls.dispose()
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