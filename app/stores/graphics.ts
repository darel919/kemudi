import { defineStore } from 'pinia'
import type { GraphicsPreset, PresetSettings } from '~/types/graphics'
import {
  DEFAULT_PRESET,
  getEffectivePreset,
  getPresetSettings,
  detectAutoPreset,
  validatePreset,
} from '~/types/graphics'
import { logDebug } from '~/utils/debug'

const STORAGE_KEY = 'kemudi:graphics-preset'

function loadPreset(): GraphicsPreset {
  try {
    const raw = localStorage.getItem(STORAGE_KEY)
    if (raw && validatePreset(raw)) return raw as GraphicsPreset
  } catch {
    // Storage unavailable: fall through
  }
  return DEFAULT_PRESET
}

function persist(preset: GraphicsPreset) {
  try {
    if (preset === DEFAULT_PRESET) {
      localStorage.removeItem(STORAGE_KEY)
    } else {
      localStorage.setItem(STORAGE_KEY, preset)
    }
  } catch {
    // Storage unavailable: ignore
  }
}

export const useGraphicsStore = defineStore('graphics', {
  state: () => ({
    userPreset: loadPreset(),
    detectedCapability: detectAutoPreset(),
  }),
  getters: {
    effectivePreset: (state) => getEffectivePreset(state.userPreset),
    settings: (state): PresetSettings => getPresetSettings(state.userPreset),
    isAuto: (state) => state.userPreset === DEFAULT_PRESET,
  },
  actions: {
    setPreset(preset: GraphicsPreset) {
      if (!validatePreset(preset)) return
      this.userPreset = preset
      persist(preset)
      logDebug('graphics:preset-selected', { userPreset: preset, effective: this.effectivePreset })
    },
    resetToAuto() {
      this.userPreset = DEFAULT_PRESET
      persist(DEFAULT_PRESET)
      logDebug('graphics:preset-selected', { userPreset: 'auto', effective: this.effectivePreset })
    },
    detectCapability() {
      this.detectedCapability = detectAutoPreset()
      logDebug('graphics:capability-detected', {
        autoSelection: this.detectedCapability,
        userOverride: this.userPreset,
      })
    },
    $reset() {
      this.userPreset = DEFAULT_PRESET
      this.detectedCapability = detectAutoPreset()
      localStorage.removeItem(STORAGE_KEY)
    },
  },
})
