<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from "vue"
import { useThreeScene, type RendererConfig } from "@/composables/useThreeScene"

const props = defineProps<{ rendererConfig: RendererConfig }>()

const canvasRef = ref<HTMLElement | null>(null)
let threeHandle: ReturnType<typeof useThreeScene> | null = null

onMounted(() => {
  if (!canvasRef.value) return
  threeHandle = useThreeScene(canvasRef.value, props.rendererConfig)
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
  <div ref="canvasRef" class="three-canvas-container" />
</template>

<style scoped>
.three-canvas-container {
  width: 100%;
  height: 100%;
  position: relative;
}
.three-canvas-container > canvas {
  display: block;
  width: 100%;
  height: 100%;
}
</style>