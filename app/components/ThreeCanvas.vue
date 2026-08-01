<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from "vue"
import { useThreeScene, type RendererConfig } from "@/composables/useThreeScene"

const props = defineProps<{ rendererConfig: RendererConfig }>()

const canvasRef = ref<HTMLCanvasElement | null>(null)
let threeHandle: ReturnType<typeof useThreeScene> | null = null

onMounted(() => {
  if (!canvasRef.value) return
  threeHandle = useThreeScene(canvasRef.value, props.rendererConfig)
  threeHandle.startLoop(() => {})
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
