import { ref } from 'vue'
import { storeToRefs } from 'pinia'
import type { GraphicsPreset } from '~/types/graphics'
import { GRAPHICS_PRESETS, detectGraphicsCapabilities } from '~/types/graphics'
import { useGraphicsStore } from '~/stores/graphics'
import { logDebug } from '~/utils/debug'

/** Shared graphics settings facade backed by the Pinia store. */
export function useGraphicsSettings() {
  const store = useGraphicsStore()
  const { userPreset, effectivePreset, settings, isAuto } = storeToRefs(store)
  const isLoaded = ref(typeof window !== 'undefined')

  if (typeof window !== 'undefined') {
    store.detectCapability()
    isLoaded.value = true
    logDebug('graphics:capability-detected', {
      autoSelection: store.detectedCapability,
      userOverride: userPreset.value,
      capabilities: detectGraphicsCapabilities(),
    })
  }

  function setPreset(preset: GraphicsPreset) {
    store.setPreset(preset)
  }

  function resetToAuto() {
    store.resetToAuto()
  }

  function getAllPresets() {
    return {
      user: userPreset.value,
      effective: effectivePreset.value,
      settings: GRAPHICS_PRESETS,
    }
  }

  return {
    userPreset,
    effectivePreset,
    settings,
    isAuto,
    isLoaded,
    setPreset,
    resetToAuto,
    getAllPresets,
  }
}
