<script setup lang="ts">
import { useGraphicsSettings } from '~/composables/useGraphicsSettings'
import type { GraphicsPreset } from '~/types/graphics'

const { userPreset, effectivePreset, settings, isAuto, setPreset, resetToAuto } =
  useGraphicsSettings()

const presets: GraphicsPreset[] = ['auto', 'low', 'medium', 'high']

function handleChange(value: GraphicsPreset) {
  setPreset(value)
}
</script>

<template>
  <div class="graphics-settings">
    <label for="preset-select" class="graphics-settings__label">
      Graphics Quality
    </label>
    <select
      id="preset-select"
      :value="userPreset"
      class="graphics-settings__select"
      @change="handleChange(($event.target as HTMLSelectElement).value as GraphicsPreset)"
    >
      <option v-for="preset in presets" :key="preset" :value="preset">
        {{ preset.charAt(0).toUpperCase() + preset.slice(1) }}
      </option>
    </select>

    <button
      v-if="!isAuto"
      class="graphics-settings__reset"
      type="button"
      @click="resetToAuto()"
    >
      Reset to Auto
    </button>

    <div class="graphics-settings__effective">
      Effective: <strong>{{ effectivePreset }}</strong>
    </div>
  </div>
</template>

<style scoped>
.graphics-settings {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.5rem 1rem;
  background: rgba(0, 0, 0, 0.6);
  border-radius: 8px;
  color: #e0e0e0;
  font-family: system-ui, sans-serif;
  font-size: 0.875rem;
}

.graphics-settings__label {
  font-weight: 500;
  white-space: nowrap;
}

.graphics-settings__select {
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  border: 1px solid #555;
  background: #1a1a2e;
  color: #e0e0e0;
  font-size: 0.875rem;
  cursor: pointer;
}

.graphics-settings__reset {
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  border: 1px solid #666;
  background: #2a2a3e;
  color: #ccc;
  font-size: 0.75rem;
  cursor: pointer;
}

.graphics-settings__reset:hover {
  background: #3a3a4e;
}

.graphics-settings__effective {
  font-size: 0.75rem;
  color: #aaa;
  white-space: nowrap;
}
</style>