import { defineStore } from 'pinia'

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
    hasWarnings: (state) => state.allSignals.some(s => s.warning || s.fault),
  },

  actions: {
    updateSignal(id: string, value: number | string | boolean, quality: 'good' | 'stale' | 'invalid' = 'good') {
      const existing = this.signals.get(id)
      if (!existing) {
        console.warn(`[telemetry] Unknown signal id: ${id}`)
        return
      }
      const now = Date.now()
      this.signals.set(id, { ...existing, value, quality, timestamp: now })

      // Append to history if numeric
      if (typeof value === 'number') {
        const hist = this.history.get(id) ?? []
        hist.push({ value, timestamp: now })
        if (hist.length > this.maxHistoryLength) hist.shift()
        this.history.set(id, hist)
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

    clearHistory() {
      this.history.clear()
    },

    $reset() {
      this.signals.clear()
      this.history.clear()
    },
  },
})