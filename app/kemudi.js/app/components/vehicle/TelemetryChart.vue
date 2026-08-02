<script setup lang="ts">
import { computed, onMounted, ref, watch } from 'vue'
import { useTelemetryStore } from '~/stores/telemetry'
import { VIRTUAL_OBD_PIDS, type TelemetrySignalDefinition } from '~/types/telemetry'
import { getChartRange, interpolateCurve, toSvgPolyline, type ChartPoint, type ChartRange } from '~/utils/telemetryChart'

type ChartMode = 'response' | 'history'

const props = defineProps<{
  rpm: number
  engineTorqueCurve: readonly (readonly [number, number])[]
}>()

const store = useTelemetryStore()
const mode = ref<ChartMode>('response')
const selectedSignalId = ref('vehicle.acceleration')
const mounted = ref(false)

const STORAGE_KEY = 'kemudi.telemetry-chart'
const chartWidth = 440
const chartHeight = 150

const chartableDefs = computed<TelemetrySignalDefinition[]>(() =>
  VIRTUAL_OBD_PIDS.filter(def => def.minValue !== undefined && def.maxValue !== undefined),
)

const selectedDef = computed(() =>
  chartableDefs.value.find(def => def.id === selectedSignalId.value) ?? chartableDefs.value[0],
)

const responsePoints = computed<ChartPoint[]>(() =>
  props.engineTorqueCurve.map(([x, y]) => ({ x, y })),
)

const responseXRange = computed<ChartRange>(() =>
  getChartRange(responsePoints.value.map(point => point.x), { min: 0, max: 9000 }, 0),
)

const responseYRange = computed<ChartRange>(() =>
  getChartRange(responsePoints.value.map(point => point.y), { min: 0, max: 800 }, 0.04),
)

const responsePath = computed(() =>
  toSvgPolyline(responsePoints.value, chartWidth, chartHeight, responseXRange.value, responseYRange.value),
)

const liveTorque = computed(() => interpolateCurve(props.engineTorqueCurve, props.rpm))

const historyValues = computed(() => {
  const signalHistory = store.history.get(selectedSignalId.value) ?? []
  return signalHistory
    .map(point => point.value)
    .filter((value): value is number => typeof value === 'number' && Number.isFinite(value))
})

const historyRange = computed<ChartRange>(() => {
  const definition = selectedDef.value
  return getChartRange(
    historyValues.value,
    { min: definition?.minValue ?? 0, max: definition?.maxValue ?? 1 },
  )
})

const historyPoints = computed<ChartPoint[]>(() =>
  historyValues.value.map((y, x) => ({ x, y })),
)

const historyPath = computed(() =>
  toSvgPolyline(
    historyPoints.value,
    chartWidth,
    chartHeight,
    { min: 0, max: Math.max(historyPoints.value.length - 1, 1) },
    historyRange.value,
  ),
)

const currentValue = computed(() => {
  if (mode.value === 'response') return liveTorque.value
  const signal = store.signals.get(selectedSignalId.value)
  return typeof signal?.value === 'number' && Number.isFinite(signal.value) ? signal.value : null
})

const currentDisplay = computed(() => currentValue.value === null ? '—' : Math.round(currentValue.value).toString())

function loadPreferences() {
  if (!import.meta.client) return
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? '{}') as { mode?: ChartMode; signalId?: string }
    if (saved.mode === 'response' || saved.mode === 'history') mode.value = saved.mode
    if (saved.signalId && chartableDefs.value.some(def => def.id === saved.signalId)) selectedSignalId.value = saved.signalId
  } catch {
    // Invalid local preferences should not prevent the HUD from rendering.
  }
}

function savePreferences() {
  if (!mounted.value || !import.meta.client) return
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({ mode: mode.value, signalId: selectedSignalId.value }))
  } catch {
    // Storage can be unavailable in private or restricted browsing contexts.
  }
}

watch([mode, selectedSignalId], savePreferences)

onMounted(() => {
  store.registerAllSignals(VIRTUAL_OBD_PIDS)
  mounted.value = true
  loadPreferences()
})
</script>

<template>
  <section class="telemetry-chart" aria-label="Configurable telemetry chart">
    <header class="telemetry-chart__header">
      <div>
        <span class="telemetry-chart__eyebrow">TELEMETRY MFD</span>
        <strong>{{ mode === 'response' ? 'ENGINE RESPONSE' : selectedDef?.label ?? 'LIVE TRACE' }}</strong>
      </div>
      <div class="telemetry-chart__controls">
        <label>
          VIEW
          <select v-model="mode" aria-label="Telemetry chart view">
            <option value="response">ENGINE MAP</option>
            <option value="history">LIVE TRACE</option>
          </select>
        </label>
        <label v-if="mode === 'history'">
          SIGNAL
          <select v-model="selectedSignalId" aria-label="Telemetry chart signal">
            <option v-for="def in chartableDefs" :key="def.id" :value="def.id">{{ def.label }}</option>
          </select>
        </label>
      </div>
    </header>

    <div class="telemetry-chart__body">
      <svg viewBox="0 0 440 150" role="img" :aria-label="mode === 'response' ? 'Engine torque response curve' : `${selectedDef?.label ?? 'Telemetry'} history`">
        <g class="chart-grid">
          <line v-for="row in 4" :key="`row-${row}`" x1="0" :y1="row * 30" x2="440" :y2="row * 30" />
          <line v-for="column in 8" :key="`column-${column}`" :x1="column * 55" y1="0" :x2="column * 55" y2="150" />
        </g>
        <polyline v-if="mode === 'response' && responsePath" class="chart-line chart-line--response" :points="responsePath" />
        <polyline v-if="mode === 'history' && historyPath" class="chart-line chart-line--history" :points="historyPath" />
        <circle
          v-if="mode === 'response' && liveTorque !== null && responseXRange.max > responseXRange.min && responseYRange.max > responseYRange.min"
          class="chart-point"
          :cx="((rpm - responseXRange.min) / (responseXRange.max - responseXRange.min)) * chartWidth"
          :cy="chartHeight - ((liveTorque - responseYRange.min) / (responseYRange.max - responseYRange.min)) * chartHeight"
          r="4"
        />
        <text x="8" y="14">{{ mode === 'response' ? `${Math.round(responseYRange.max)} N·m` : `${Math.round(historyRange.max)} ${selectedDef?.unit ?? ''}` }}</text>
        <text x="8" y="145">{{ mode === 'response' ? '0 RPM' : 'OLDER SAMPLES' }}</text>
        <text x="382" y="145">{{ mode === 'response' ? `${Math.round(responseXRange.max)} RPM` : 'NOW' }}</text>
      </svg>
      <div class="telemetry-chart__readout">
        <strong>{{ currentDisplay }}</strong>
        <span>{{ mode === 'response' ? 'N·m @ current RPM' : selectedDef?.unit || 'value' }}</span>
        <small v-if="mode === 'response'">{{ Math.round(rpm) }} RPM · calibrated vehicle map</small>
        <small v-else>{{ historyValues.length }} samples · {{ store.maxHistoryLength }} max</small>
      </div>
    </div>

    <p v-if="mode === 'response' && !responsePath" class="telemetry-chart__empty">No engine response curve is configured for this vehicle.</p>
    <p v-if="mode === 'history' && !historyPath" class="telemetry-chart__empty">Collecting samples…</p>
  </section>
</template>

<style scoped>
.telemetry-chart { width: min(460px, 42vw); min-width: 330px; padding: 11px 13px 10px; color: #e7f1f0; background: rgba(8, 14, 23, .88); border: 1px solid rgba(153, 174, 207, .25); border-radius: 9px; box-shadow: 0 8px 30px rgba(0,0,0,.25); backdrop-filter: blur(10px); pointer-events: auto; }
.telemetry-chart__header { display: flex; justify-content: space-between; align-items: flex-start; gap: 16px; }
.telemetry-chart__header > div:first-child { display: grid; gap: 3px; }
.telemetry-chart__eyebrow, .telemetry-chart__controls label { color: #6ee7c5; font-size: 8px; font-weight: 800; letter-spacing: .13em; }
.telemetry-chart__header strong { font-size: 12px; letter-spacing: .04em; }
.telemetry-chart__controls { display: flex; justify-content: flex-end; gap: 6px; }
.telemetry-chart__controls label { display: grid; gap: 3px; color: #74849a; }
.telemetry-chart select { max-width: 118px; padding: 3px 5px; color: #dbe8e8; background: #111d2a; border: 1px solid rgba(153,174,207,.25); border-radius: 3px; font: 800 8px Inter, ui-sans-serif, system-ui, sans-serif; letter-spacing: .04em; }
.telemetry-chart__body { display: flex; align-items: center; gap: 10px; margin-top: 8px; }
.telemetry-chart svg { flex: 1; min-width: 0; height: auto; overflow: visible; }
.chart-grid line { stroke: rgba(174, 198, 214, .15); stroke-width: 1; }
.chart-line { fill: none; stroke-width: 2.25; stroke-linecap: round; stroke-linejoin: round; }
.chart-line--response { stroke: #f5d66d; filter: drop-shadow(0 0 4px rgba(245,214,109,.35)); }
.chart-line--history { stroke: #6ee7c5; filter: drop-shadow(0 0 4px rgba(110,231,197,.35)); }
.chart-point { fill: #ff858d; stroke: #fff4f2; stroke-width: 1.5; filter: drop-shadow(0 0 5px rgba(255,133,141,.7)); }
svg text { fill: #77879a; font: 8px Inter, ui-sans-serif, system-ui, sans-serif; letter-spacing: .04em; }
.telemetry-chart__readout { min-width: 90px; display: grid; gap: 3px; text-align: right; }
.telemetry-chart__readout strong { color: #f4f8fb; font-size: 24px; line-height: 1; }
.telemetry-chart__readout span, .telemetry-chart__readout small { color: #8998a9; font-size: 8px; line-height: 1.25; }
.telemetry-chart__readout small { color: #6ee7c5; }
.telemetry-chart__empty { margin: 8px 0 0; color: #9baabd; font-size: 9px; }
@media (max-width: 900px) { .telemetry-chart { width: min(430px, 52vw); min-width: 280px; }.telemetry-chart__body { display: block; }.telemetry-chart__readout { display: flex; align-items: baseline; justify-content: flex-end; gap: 4px; }.telemetry-chart__readout strong { font-size: 17px; } }
@media (max-width: 600px) { .telemetry-chart { display: none; } }
</style>
