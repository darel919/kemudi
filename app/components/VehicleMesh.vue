<script setup lang="ts">
import { onBeforeUnmount, shallowRef, watch } from 'vue'
import * as THREE from 'three'
import { useVehicleSkinning } from '~/composables/useVehicleSkinning'
import { logDebug } from '~/utils/debug'

const props = defineProps<{
  positions: Float64Array
  velocities?: Float64Array
  scene?: THREE.Scene
}>()

// Rest node positions for a unit box (8 corners)
const HALF = 0.5
const restPositions = new Float64Array([
  -HALF, -HALF, +HALF,  // 0
  +HALF, -HALF, +HALF,  // 1
  -HALF, -HALF, -HALF,  // 2
  +HALF, -HALF, -HALF,  // 3
  -HALF, +HALF, +HALF,  // 4
  +HALF, +HALF, +HALF,  // 5
  -HALF, +HALF, -HALF,  // 6
  +HALF, +HALF, -HALF,  // 7
])

const geometry = new THREE.BoxGeometry(1, 1, 1)
const material = new THREE.MeshStandardMaterial({
  color: 0x4488cc,
  roughness: 0.6,
  metalness: 0.3,
})
const mesh = new THREE.Mesh(geometry, material)
const meshRef = shallowRef<THREE.Mesh | null>(mesh)

const skinning = useVehicleSkinning(geometry, restPositions)

watch(
  () => props.positions,
  (p) => { if (p) skinning.update(p) },
  { immediate: true },
)

onMounted(() => {
  props.scene?.add(mesh)
  logDebug('vehicle-mesh:mounted', {
    vertexCount: geometry.getAttribute('position').count,
  })
})

onBeforeUnmount(() => {
  props.scene?.remove(mesh)
  skinning.dispose()
  geometry.dispose()
  material.dispose()
  meshRef.value = null
  logDebug('vehicle-mesh:unmounted', {})
})

defineExpose({ mesh })
</script>

<template>
  <slot />
</template>
