<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch } from "vue"
import { useThreeScene, type RendererConfig } from "@/composables/useThreeScene"

const props = withDefaults(defineProps<{
  rendererConfig: RendererConfig
  controlsEnabled?: boolean
}>(), {
  controlsEnabled: true,
})

const canvasRef = ref<HTMLCanvasElement | null>(null)
let threeHandle: ReturnType<typeof useThreeScene> | null = null

onMounted(() => {
  if (!canvasRef.value) return
  threeHandle = useThreeScene(canvasRef.value, props.rendererConfig)
  threeHandle.controls.enabled = props.controlsEnabled
  threeHandle.startLoop(() => {})
})

watch(() => props.controlsEnabled, (enabled) => {
  if (threeHandle) threeHandle.controls.enabled = enabled
})

onBeforeUnmount(() => {
  threeHandle?.dispose()
  threeHandle = null
})

defineExpose({
  getScene() {
    return threeHandle?.scene ?? null
  },
  getCamera() {
    return threeHandle?.camera ?? null
  },
  getRenderer() {
    return threeHandle?.renderer ?? null
  },
})
</script>

<template>
  <canvas ref="canvasRef" class="three-canvas" aria-label="3D vehicle scene" />
</template>

<style scoped>
.three-canvas {
  width: 100%;
  height: 100%;
  display: block;
  position: relative;
}
</style>
