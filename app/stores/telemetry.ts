import { defineStore } from 'pinia'
import type { TelemetrySignalDefinition } from '~/types/telemetry'

export type TelemetrySignal = {
  id: string
  label: string
  value: number | string | boolean
  unit: string
  warning?: boolean
  fault?: boolean
  timestamp: number
  quality: 'good' | 'stale' | 'invalid'
}

export const useTelemetryStore = defineStore('telemetry', {
  state: () => ({
    signals: new Map<string, TelemetrySignal>(),
    history: new Map<string, { value: number | string | boolean; timestamp: number }[]>(),
    maxHistoryLength: 300,
  }),

  getters: {
    getSignal: (state) => (id: string) => state.signals.get(id),
    allSignals: (state): TelemetrySignal[] => Array.from(state.signals.values()),
    hasWarnings: (state) => Array.from(state.signals.values()).some(s => s.warning || s.fault),
  },

  actions: {
    updateSignal(id: string, value: number | string | boolean, quality: 'good' | 'stale' | 'invalid' = 'good') {
      this.updateSnapshot([[id, value, quality]], Date.now(), true)
    },

    /**
     * Publish a coherent UI snapshot as one store action. Physics remains
     * authoritative in the worker; this intentionally samples presentation
     * telemetry instead of making Pinia reactive to every simulation tick.
     */
    updateSnapshot(
      updates: readonly (readonly [id: string, value: number | string | boolean, quality?: 'good' | 'stale' | 'invalid'])[],
      timestamp = Date.now(),
      recordHistory = false,
    ) {
      for (const [id, value, quality = 'good'] of updates) {
        const existing = this.signals.get(id)
        if (!existing) {
          continue
        }
        this.signals.set(id, { ...existing, value, quality, timestamp })

        // History is deliberately sampled more slowly than live OBD values.
        if (recordHistory && typeof value === 'number') {
          const hist = this.history.get(id) ?? []
          hist.push({ value, timestamp })
          if (hist.length > this.maxHistoryLength) hist.shift()
          this.history.set(id, hist)
        }
      }
    },

    setWarning(id: string, warning: boolean) {
      const existing = this.signals.get(id)
      if (existing) {
        this.signals.set(id, { ...existing, warning })
      }
    },

    setFault(id: string, fault: boolean) {
      const existing = this.signals.get(id)
      if (existing) {
        this.signals.set(id, { ...existing, fault })
      }
    },

    getHistory(id: string) {
      return this.history.get(id) ?? []
    },

    registerSignal(signal: TelemetrySignal) {
      this.signals.set(signal.id, signal)
    },
    registerAllSignals(defs: readonly TelemetrySignalDefinition[]) {
      for (const def of defs) {
        if (this.signals.has(def.id)) continue
        this.signals.set(def.id, {
          id: def.id,
          label: def.label,
          value: 0,
          unit: def.unit,
          warning: false,
          fault: false,
          timestamp: Date.now(),
          quality: 'good' as const,
        })
      }
    },

    clearHistory() {
      this.history.clear()
    },

    $reset() {
      this.signals.clear()
      this.history.clear()
    },
  },
})
