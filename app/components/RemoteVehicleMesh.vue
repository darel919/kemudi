<script setup lang="ts">
import { onBeforeUnmount, onMounted, shallowRef, watch } from 'vue'
import * as THREE from 'three'

const props = defineProps<{
  scene: THREE.Scene
  positions: ArrayLike<number>
  color?: number
}>()

const group = shallowRef<THREE.Group | null>(null)
let body: THREE.Mesh | null = null
let geometry: THREE.BoxGeometry | null = null
let material: THREE.MeshStandardMaterial | null = null
let targetPositions = new Float64Array(0)
let displayedPositions = new Float64Array(0)
let animationFrame = 0
let previousTime = 0

function setTargetPositions(positions: ArrayLike<number>) {
  if (positions.length < 3 || positions.length > 384) return
  if (targetPositions.length !== positions.length) {
    targetPositions = new Float64Array(positions.length)
    displayedPositions = new Float64Array(positions.length)
    for (let i = 0; i < positions.length; i++) displayedPositions[i] = Number(positions[i] ?? 0)
  }
  for (let i = 0; i < positions.length; i++) {
    const value = Number(positions[i] ?? 0)
    if (!Number.isFinite(value)) return
    targetPositions[i] = value
  }
}

function updateBounds(positions: ArrayLike<number>) {
  if (!body || positions.length < 3) return
  let minX = Infinity; let minY = Infinity; let minZ = Infinity
  let maxX = -Infinity; let maxY = -Infinity; let maxZ = -Infinity
  let centerX = 0; let centerY = 0; let centerZ = 0
  let count = 0
  for (let i = 0; i + 2 < positions.length; i += 3) {
    const x = positions[i] ?? 0
    const y = positions[i + 1] ?? 0
    const z = positions[i + 2] ?? 0
    minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z)
    maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z)
    centerX += x; centerY += y; centerZ += z; count++
  }
  if (!count) return
  body.position.set(centerX / count, centerY / count, centerZ / count)
  body.scale.set(Math.max(0.5, maxX - minX), Math.max(0.35, maxY - minY), Math.max(0.8, maxZ - minZ))
}

onMounted(() => {
  group.value = new THREE.Group()
  geometry = new THREE.BoxGeometry(1, 1, 1)
  material = new THREE.MeshStandardMaterial({ color: props.color ?? 0xd58a4e, roughness: 0.55, metalness: 0.2 })
  body = new THREE.Mesh(geometry, material)
  body.castShadow = false
  body.receiveShadow = false
  group.value.add(body)
  props.scene.add(group.value)
  setTargetPositions(props.positions)
  updateBounds(displayedPositions)
  const animate = (now: number) => {
    if (!body) return
    animationFrame = requestAnimationFrame(animate)
    const dt = previousTime > 0 ? Math.min(0.1, Math.max(0.001, (now - previousTime) / 1000)) : 1 / 60
    previousTime = now
    if (displayedPositions.length === targetPositions.length && displayedPositions.length > 0) {
      const blend = 1 - Math.exp(-dt / 0.075)
      let changed = false
      for (let i = 0; i < displayedPositions.length; i++) {
        const current = displayedPositions[i] ?? 0
        const target = targetPositions[i] ?? current
        const next = current + (target - current) * blend
        changed ||= Math.abs(next - current) > 1e-5
        displayedPositions[i] = next
      }
      if (changed) updateBounds(displayedPositions)
    }
  }
  animationFrame = requestAnimationFrame(animate)
})

watch(() => props.positions, setTargetPositions)

onBeforeUnmount(() => {
  cancelAnimationFrame(animationFrame)
  if (group.value) props.scene.remove(group.value)
  geometry?.dispose()
  material?.dispose()
  body = null
  group.value = null
})
</script>

<template />
