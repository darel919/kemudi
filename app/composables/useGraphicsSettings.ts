import { ref, computed, watch } from 'vue'
import type { GraphicsPreset, PresetSettings } from '~/types/graphics'
import {
  DEFAULT_PRESET,
  GRAPHICS_PRESETS,
  getEffectivePreset,
  getPresetSettings,
  detectAutoPreset,
  validatePreset,
} from '~/types/graphics'
import { logDebug } from '~/utils/debug'

const STORAGE_KEY = 'kemudi:graphics-preset'

const userPreset = ref<GraphicsPreset>(DEFAULT_PRESET)
const isLoaded = ref(false)

function loadStoredPreset(): GraphicsPreset {
  if (typeof window === 'undefined') return DEFAULT_PRESET
  try {
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored && validatePreset(stored)) {
      return stored as GraphicsPreset
    }
  } catch {
    // localStorage unavailable — fall back to default
  }
  return DEFAULT_PRESET
}

function persistPreset(preset: GraphicsPreset) {
  if (typeof window === 'undefined') return
  try {
    localStorage.setItem(STORAGE_KEY, preset)
  } catch {
    // silently skip if storage unavailable
  }
}

if (typeof window !== 'undefined') {
  userPreset.value = loadStoredPreset()
  isLoaded.value = true

  const detected = detectAutoPreset()
  logDebug('graphics:capability-detected', {
    autoSelection: detected,
    userOverride: userPreset.value,
  })
}

watch(userPreset, (newVal) => {
  if (typeof window === 'undefined') return
  if (newVal === DEFAULT_PRESET) {
    try {
      localStorage.removeItem(STORAGE_KEY)
    } catch {
      // ignore
    }
  } else {
    persistPreset(newVal)
  }
  const effective = getEffectivePreset(newVal)
  logDebug('graphics:preset-selected', { userPreset: newVal, effective })
})

export function useGraphicsSettings() {
  const effectivePreset = computed(() => getEffectivePreset(userPreset.value))
  const settings = computed<PresetSettings>(() => getPresetSettings(userPreset.value))
  const isAuto = computed(() => userPreset.value === 'auto')

  function setPreset(preset: GraphicsPreset) {
    if (!validatePreset(preset)) return
    userPreset.value = preset
  }

  function resetToAuto() {
    userPreset.value = DEFAULT_PRESET
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