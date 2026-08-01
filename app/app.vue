<template>
  <div class="fullscreen">
    <NuxtRouteAnnouncer />
    <ThreeCanvas :rendererConfig="rendererConfig" />
  </div>
</template>

<script setup lang="ts">
const rendererConfig = {
  pixelRatioCap: 2,
  antialias: true,
  shadows: true,
  toneMappingExposure: 1.1,
}
</script>

<style>
.fullscreen {
  width: 100%;
  height: 100vh;
  position: relative;
}
</style>