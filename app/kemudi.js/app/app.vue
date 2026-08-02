<template>
  <div class="app-shell">
    <NuxtRouteAnnouncer />
    <ThreeCanvas
      ref="canvasRef"
      :rendererConfig="rendererConfig"
      :controls-enabled="menuOpen"
    />
    <DrivingScene
      v-if="!menuOpen && vehicleSession.vehicle && vehicleSession.map && scene && camera"
      :scene="scene"
      :camera="camera"
      :vehicle-path="vehicleSession.vehicle.assetPath"
      :vehicle-name="vehicleSession.vehicle.name"
      :mode-label="playModeStore.currentMode.label"
      :transmission="vehicleSession.vehicle.transmission"
      :map="vehicleSession.map"
      @menu="menuOpen = true"
    />
    <div class="menu-layer" :class="{ 'menu-layer--closed': !menuOpen }">
      <MainMenu v-if="menuOpen" @start="menuOpen = false" />
    </div>
  </div>
</template>

<script setup lang="ts">
import { onMounted, onUnmounted, ref, shallowRef } from 'vue'
import * as THREE from 'three'
import { usePlayModeStore } from '~/stores/playMode'
import { useVehicleSessionStore } from '~/stores/vehicleSession'

const { settings } = useGraphicsSettings()
const playModeStore = usePlayModeStore()
const vehicleSession = useVehicleSessionStore()
const menuOpen = ref(true)
const canvasRef = ref<{
  getScene: () => THREE.Scene | null
  getCamera: () => THREE.PerspectiveCamera | null
} | null>(null)
const scene = shallowRef<THREE.Scene | null>(null)
const camera = shallowRef<THREE.PerspectiveCamera | null>(null)

const rendererConfig = computed(() => ({
  pixelRatioCap: settings.value.pixelRatioCap,
  antialias: settings.value.antialias,
  shadows: settings.value.shadows,
  toneMappingExposure: settings.value.toneMappingExposure,
  shadowMapSize: settings.value.shadowMapSize,
  renderPixelBudget: 4_500_000,
}))

function toggleMenu(event: KeyboardEvent) {
  if (event.key === 'Escape' && (!menuOpen.value || vehicleSession.hasVehicle)) {
    menuOpen.value = !menuOpen.value
  }
}

onMounted(() => {
  scene.value = canvasRef.value?.getScene() ?? null
  camera.value = canvasRef.value?.getCamera() ?? null
  window.addEventListener('keydown', toggleMenu)
})
onUnmounted(() => window.removeEventListener('keydown', toggleMenu))
</script>

<style>
.app-shell {
  width: 100%;
  height: 100vh;
  position: relative;
  overflow: hidden;
  background: #20212b;
}

.menu-layer {
  position: absolute;
  inset: 0;
  z-index: 10;
  display: grid;
  place-items: center;
  padding: 24px;
  background: rgba(5, 8, 15, .42);
  backdrop-filter: blur(7px);
}

.menu-layer--closed {
  pointer-events: none;
  background: transparent;
  backdrop-filter: none;
}

@media (max-width: 600px) {
  .menu-layer { padding: 10px; }
}
</style>
