<template>
  <div class="fullscreen">
    <NuxtRouteAnnouncer />
    <ThreeCanvas :rendererConfig="rendererConfig" />
  </div>
</template>

<script setup lang="ts">
const { settings } = useGraphicsSettings()
const rendererConfig = computed(() => ({
  pixelRatioCap: settings.value.pixelRatioCap,
  antialias: settings.value.antialias,
  shadows: settings.value.shadows,
  toneMappingExposure: settings.value.toneMappingExposure,
  shadowMapSize: settings.value.shadowMapSize,
}))
</script>

<style>
.fullscreen {
  width: 100%;
  height: 100vh;
  position: relative;
}
</style>
