<script setup lang="ts">
import type { SubsystemId, TelemetrySignalDefinition } from '~/types/telemetry'
import { VIRTUAL_OBD_PIDS } from '~/types/telemetry'
import { useTelemetryStore } from '~/stores/telemetry'

const props = withDefaults(defineProps<{
  compact?: boolean
  subsystems?: SubsystemId[]
}>(), {
  compact: true,
  subsystems: () => [] as SubsystemId[],
})

const store = useTelemetryStore()

const now = ref(Date.now())
let tickTimer: ReturnType<typeof setInterval> | undefined

onMounted(() => {
  store.registerAllSignals(VIRTUAL_OBD_PIDS)
  tickTimer = setInterval(() => { now.value = Date.now() }, 500)
})
onUnmounted(() => {
  if (tickTimer) clearInterval(tickTimer)
})

/** All definitions filtered by subsystem (empty = all) */
const visibleDefs = computed<TelemetrySignalDefinition[]>(() => {
  const all = [...VIRTUAL_OBD_PIDS]
  return props.subsystems.length
    ? all.filter(d => props.subsystems.includes(d.subsystem))
    : all
})

/** Compact mode shows only these key signals */
const COMPACT_IDS = [
  'engine.rpm',
  'vehicle.speed',
  'engine.coolant_temp',
  'engine.oil_pressure',
  'fuel.level',
  'electrical.battery_voltage',
]

const compactDefs = computed(() =>
  VIRTUAL_OBD_PIDS.filter(d => COMPACT_IDS.includes(d.id)),
)

const displayDefs = computed(() =>
  props.compact ? compactDefs.value : visibleDefs.value,
)

function isStale(timestamp: number): boolean {
  return now.value - timestamp > 2000
}

function signalColor(signalId: string): string {
  const sig = store.signals.get(signalId)
  if (!sig) return 'text-gray-500'
  if (sig.fault) return 'text-red-500'
  if (sig.warning) return 'text-amber-400'
  return 'text-emerald-400'
}

function signalBg(signalId: string): string {
  const sig = store.signals.get(signalId)
  if (!sig) return 'bg-gray-800'
  if (sig.fault) return 'bg-red-900/40 border-red-700'
  if (sig.warning) return 'bg-amber-900/30 border-amber-700'
  return 'bg-gray-800 border-gray-700'
}

function formatValue(signalId: string): string {
  const sig = store.signals.get(signalId)
  if (!sig) return '---'
  return String(sig.value)
}

/** Active warnings and faults for the panel */
const activeAlerts = computed(() => {
  return store.allSignals
    .filter(s => s.warning || s.fault)
    .sort((a, b) => (b.fault ? 1 : 0) - (a.fault ? 1 : 0))
})

/** Unique subsystems present in the visible definitions */
const subsystemLabels: Record<SubsystemId, string> = {
  engine: 'Engine',
  drivetrain: 'Drivetrain',
  suspension: 'Suspension',
  tires: 'Tires',
  body: 'Body',
  fuel: 'Fuel',
  electrical: 'Electrical',
  vehicle: 'Vehicle',
  safety: 'Safety',
}
</script>

<template>
  <div class="flex flex-col gap-4">
    <!-- Warning Panel -->
    <div
      v-if="activeAlerts.length"
      class="rounded-lg border border-amber-700 bg-amber-950/50 p-3"
    >
      <h3 class="mb-2 text-xs font-semibold uppercase tracking-wider text-amber-400">
        Active Alerts
      </h3>
      <div class="flex flex-wrap gap-2">
        <span
          v-for="alert in activeAlerts"
          :key="alert.id"
          class="inline-flex items-center gap-1.5 rounded px-2 py-0.5 text-xs font-medium"
          :class="alert.fault
            ? 'bg-red-900/60 text-red-300'
            : 'bg-amber-900/60 text-amber-300'"
        >
          <span
            class="inline-block h-1.5 w-1.5 rounded-full"
            :class="alert.fault ? 'bg-red-400' : 'bg-amber-400'"
          />
          {{ alert.label }}
        </span>
      </div>
    </div>

    <!-- Gauge Grid — grouped by subsystem in expanded mode -->
    <template v-if="!compact">
      <div
        v-for="subsystem in [...new Set(visibleDefs.map(d => d.subsystem))]"
        :key="subsystem"
      >
        <h3 class="mb-2 text-xs font-semibold uppercase tracking-wider text-gray-400">
          {{ subsystemLabels[subsystem as SubsystemId] ?? subsystem }}
        </h3>
        <div class="grid grid-cols-2 gap-2 sm:grid-cols-3 md:grid-cols-4 lg:grid-cols-6">
          <div
            v-for="def in visibleDefs.filter(d => d.subsystem === subsystem)"
            :key="def.id"
            class="flex flex-col rounded-lg border p-3 transition-colors"
            :class="signalBg(def.id)"
          >
            <span class="text-[10px] font-medium uppercase tracking-wider text-gray-400">
              {{ def.label }}
            </span>
            <span
              class="mt-1 text-lg font-bold tabular-nums"
              :class="signalColor(def.id)"
            >
              {{ formatValue(def.id) }}
              <span
                v-if="store.signals.has(def.id)"
                class="text-xs font-normal text-gray-500"
              >{{ def.unit }}</span>
            </span>
            <!-- Stale indicator -->
            <span
              v-if="store.signals.has(def.id) && isStale(store.signals.get(def.id)!.timestamp)"
              class="mt-0.5 text-[9px] font-bold uppercase tracking-widest text-orange-500"
            >
              STALE
            </span>
            <!-- Unregistered indicator -->
            <span
              v-if="!store.signals.has(def.id)"
              class="mt-0.5 text-[9px] font-bold uppercase tracking-widest text-gray-600"
            >
              NO DATA
            </span>
          </div>
        </div>
      </div>
    </template>

    <!-- Compact mode: flat row -->
    <div
      v-else
      class="grid grid-cols-3 gap-2 sm:grid-cols-6"
    >
      <div
        v-for="def in compactDefs"
        :key="def.id"
        class="flex flex-col rounded-lg border p-2 transition-colors"
        :class="signalBg(def.id)"
      >
        <span class="text-[9px] font-medium uppercase tracking-wider text-gray-400">
          {{ def.label }}
        </span>
        <span
          class="mt-0.5 text-base font-bold tabular-nums"
          :class="signalColor(def.id)"
        >
          {{ formatValue(def.id) }}
          <span
            v-if="store.signals.has(def.id)"
            class="text-[10px] font-normal text-gray-500"
          >{{ def.unit }}</span>
        </span>
        <span
          v-if="store.signals.has(def.id) && isStale(store.signals.get(def.id)!.timestamp)"
          class="text-[8px] font-bold uppercase text-orange-500"
        >
          STALE
        </span>
        <span
          v-if="!store.signals.has(def.id)"
          class="text-[8px] font-bold uppercase text-gray-600"
        >
          ---
        </span>
      </div>
    </div>
  </div>
</template>

<style scoped>
/* ponytail: no custom styles needed; Tailwind handles everything. */
</style>
