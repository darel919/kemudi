<script setup lang="ts">
import { computed, ref } from 'vue'
import type { GraphicsPreset } from '~/types/graphics'
import type { PlayModeId } from '~/types/play-mode'
import type { BaseMapId } from '~/types/base-map'
import { BASE_MAPS } from '~/types/base-map'
import type { TransmissionMode, VehicleConfig } from '~/stores/vehicleSession'
import { useGraphicsSettings } from '~/composables/useGraphicsSettings'
import { usePlayMode } from '~/composables/usePlayMode'
import { useVehicleLoader } from '~/composables/useVehicleLoader'
import { usePlayModeStore } from '~/stores/playMode'
import { useVehicleSessionStore } from '~/stores/vehicleSession'
import { useMultiplayerStore } from '~/stores/multiplayer'
import GraphicsSettings from '~/components/GraphicsSettings.vue'
import TelemetryDashboard from '~/components/vehicle/TelemetryDashboard.vue'

const emit = defineEmits<{ start: [] }>()

type MenuTab = 'drive' | 'garage' | 'telemetry' | 'settings'

interface VehicleOption extends VehicleConfig {
  description: string
  drivetrain: string
  transmissionOptions: TransmissionMode[]
}

const vehicles: VehicleOption[] = [
  {
    id: 'premium-sportscar',
    name: 'Premium Sportscar',
    assetPath: '/vehicles/premium_sportscar.vehicle.json',
    transmission: 'automatic',
    description: 'A fast, responsive road car with a limited-slip differential.',
    drivetrain: 'RWD · LIMITED-SLIP DIFFERENTIAL',
    transmissionOptions: ['manual', 'automatic'],
  },
  {
    id: 'basic-truck',
    name: 'Basic Truck',
    assetPath: '/vehicles/basic_truck.vehicle.json',
    transmission: 'automatic',
    description: 'A stable automatic work truck built for forgiving handling.',
    drivetrain: 'RWD · LOCKED DIFFERENTIAL',
    transmissionOptions: ['automatic'],
  },
  {
    id: 'basic-atv',
    name: 'Basic ATV',
    assetPath: '/vehicles/basic_atv.vehicle.json',
    transmission: 'manual',
    description: 'A lightweight off-road quad for quick experiments.',
    drivetrain: 'RWD · LOCKED DIFFERENTIAL',
    transmissionOptions: ['manual'],
  },
]

const tabs: { id: MenuTab; label: string; icon: string }[] = [
  { id: 'drive', label: 'Drive', icon: '▶' },
  { id: 'garage', label: 'Garage', icon: '▣' },
  { id: 'telemetry', label: 'Telemetry', icon: '⌁' },
  { id: 'settings', label: 'Settings', icon: '⚙' },
]

const activeTab = ref<MenuTab>('drive')
const selectedVehicleId = ref('premium-sportscar')
const selectedTransmission = ref<TransmissionMode>('automatic')
const selectedMode = ref<PlayModeId>('freeroam')
const selectedMapId = ref<BaseMapId>('flat')
const isLoading = ref(false)
const errorMessage = ref('')

const vehicleLoader = useVehicleLoader()
const vehicleSession = useVehicleSessionStore()
const playModeStore = usePlayModeStore()
const playMode = usePlayMode()
const { effectivePreset, setPreset } = useGraphicsSettings()
const multiplayerStore = useMultiplayerStore()
const multiplayerEnabled = ref(multiplayerStore.multiplayerEnabled)
const multiplayerServerUrl = ref(multiplayerStore.configuredServerUrl)
const multiplayerRoomId = ref(multiplayerStore.configuredRoomId)

const selectedVehicle = computed(() =>
  vehicles.find(vehicle => vehicle.id === selectedVehicleId.value) ?? vehicles[0]!,
)

const selectedModeLabel = computed(() =>
  selectedMode.value === 'drag-race' ? 'Drag Race' : 'Freeroam',
)

const maps = Object.values(BASE_MAPS)

function chooseVehicle(vehicle: VehicleOption) {
  selectedVehicleId.value = vehicle.id
  selectedTransmission.value = vehicle.transmissionOptions.includes(vehicle.transmission)
    ? vehicle.transmission
    : vehicle.transmissionOptions[0] ?? vehicle.transmission
  errorMessage.value = ''
}

function chooseTransmission(mode: TransmissionMode) {
  if (selectedVehicle.value.transmissionOptions.includes(mode)) selectedTransmission.value = mode
}

function chooseMode(mode: PlayModeId) {
  selectedMode.value = mode
}

function chooseMap(mapId: BaseMapId) {
  selectedMapId.value = mapId
}

async function startDriving() {
  if (isLoading.value) return
  isLoading.value = true
  errorMessage.value = ''

  try {
    await vehicleLoader.loadFromUrl(selectedVehicle.value.assetPath)
    vehicleSession.spawnVehicle(
      { ...selectedVehicle.value, transmission: selectedTransmission.value },
      selectedMapId.value,
    )
    playMode.resetMode()
    playMode.enterMode(selectedMode.value)
    emit('start')
  } catch (error) {
    errorMessage.value = error instanceof Error ? error.message : 'Unable to load vehicle'
  } finally {
    isLoading.value = false
  }
}

function resetSession() {
  vehicleSession.resetSession()
  playModeStore.reset()
}

function saveMultiplayerSettings() {
  multiplayerStore.configure(multiplayerEnabled.value, multiplayerServerUrl.value, multiplayerRoomId.value)
}
</script>

<template>
  <section class="main-menu" aria-label="Kemudi main menu">
    <header class="menu-header">
      <div class="brand-lockup">
        <span class="brand-mark">K</span>
        <div>
          <p class="brand-name">KEMUDI</p>
          <p class="brand-subtitle">VEHICLE SANDBOX</p>
        </div>
      </div>
      <div class="build-status"><span class="status-dot" /> SYSTEM READY</div>
    </header>

    <nav class="menu-tabs" aria-label="Main navigation">
      <button
        v-for="tab in tabs"
        :key="tab.id"
        type="button"
        class="menu-tab"
        :class="{ 'menu-tab--active': activeTab === tab.id }"
        @click="activeTab = tab.id"
      >
        <span class="menu-tab__icon">{{ tab.icon }}</span>
        {{ tab.label }}
      </button>
    </nav>

    <main class="menu-content">
      <template v-if="activeTab === 'drive'">
        <div class="drive-layout">
          <aside class="drive-overview">
            <div class="drive-overview__eyebrow">DRIVER CONFIGURATION / 01</div>
            <div class="drive-overview__gauge">
              <span class="drive-overview__gauge-ring">K</span>
              <span><strong>READY TO RUN</strong><small>PHYSICS SANDBOX</small></span>
            </div>
            <h1>{{ selectedVehicle.name }}</h1>
            <p class="lead">{{ selectedVehicle.description }}</p>
            <div class="drive-readout">
              <div><span>SESSION</span><strong>{{ selectedModeLabel.toUpperCase() }}</strong><small>DRIVE MODE</small></div>
              <div><span>ROUTE</span><strong>{{ BASE_MAPS[selectedMapId].label }}</strong><small>{{ BASE_MAPS[selectedMapId].surfaceLabel }}</small></div>
              <div><span>GEARBOX</span><strong>{{ selectedTransmission === 'manual' ? 'MANUAL' : 'AUTO' }}</strong><small>{{ selectedTransmission === 'manual' ? 'SEQUENTIAL CONTROL' : 'TORQUE CONVERTER' }}</small></div>
            </div>
            <div class="drive-overview__vehicle-line"><i />{{ selectedVehicle.drivetrain }}</div>
            <div class="drive-overview__note"><span>INPUT CHECK</span><strong>WASD / GAMEPAD ONLINE</strong><small>E ignition · {{ selectedTransmission === 'manual' ? 'Q down / R up' : 'TCM shifts automatically' }}<template v-if="selectedVehicle.transmissionOptions.length > 1"> · T gearbox mode</template></small></div>
          </aside>

          <section class="drive-config">
            <div class="eyebrow">SELECT YOUR EXPERIENCE</div>
            <h2>Set the car up.</h2>
            <p class="lead">Choose a route, driving mode, and transmission before you roll.</p>

            <div class="section-heading section-heading--compact">
              <span>DRIVE MODE</span>
              <span class="section-heading__count">2 OPTIONS</span>
            </div>
            <div class="mode-grid">
              <button
                type="button"
                class="mode-card"
                :class="{ 'mode-card--selected': selectedMode === 'freeroam' }"
                @click="chooseMode('freeroam')"
              >
                <span class="mode-card__number">01 / OPEN</span>
                <span class="mode-card__title">Freeroam</span>
                <span class="mode-card__copy">Explore, tune handling, and deform the vehicle freely.</span>
              </button>
              <button
                type="button"
                class="mode-card"
                :class="{ 'mode-card--selected': selectedMode === 'drag-race' }"
                @click="chooseMode('drag-race')"
              >
                <span class="mode-card__number">02 / TIME</span>
                <span class="mode-card__title">Drag Race</span>
                <span class="mode-card__copy">Stage at the grid for an exact one-mile timed run.</span>
              </button>
            </div>

            <div class="section-heading section-heading--map">
              <span>BASE MAP / ROUTE</span>
              <span class="section-heading__count">{{ maps.length }} AVAILABLE</span>
            </div>
            <div class="map-grid">
              <button
                v-for="map in maps"
                :key="map.id"
                type="button"
                class="map-card"
                :class="{ 'map-card--selected': selectedMapId === map.id }"
                @click="chooseMap(map.id)"
              >
                <span class="map-card__preview" :class="`map-card__preview--${map.id}`"><i /><b /></span>
                <span class="map-card__name">{{ map.label }}</span>
                <span class="map-card__surface">{{ map.surfaceLabel }}</span>
                <span class="map-card__copy">{{ map.description }}</span>
              </button>
            </div>

            <div class="section-heading">
              <span>VEHICLE / LOADOUT</span>
              <span class="section-heading__count">{{ vehicles.length }} AVAILABLE</span>
            </div>
            <div class="vehicle-grid">
              <button
                v-for="vehicle in vehicles"
                :key="vehicle.id"
                type="button"
                class="vehicle-card"
                :class="{ 'vehicle-card--selected': selectedVehicleId === vehicle.id }"
                @click="chooseVehicle(vehicle)"
              >
                <span class="vehicle-card__silhouette">▰</span>
                <span class="vehicle-card__name">{{ vehicle.name }}</span>
                <span class="vehicle-card__drive">{{ vehicle.drivetrain }}</span>
              </button>
            </div>

            <div class="section-heading section-heading--transmission">
              <span>TRANSMISSION / DRIVER CONTROL</span>
              <span class="section-heading__count">{{ selectedVehicle.name.toUpperCase() }}</span>
            </div>
            <div class="transmission-choice" :class="{ 'transmission-choice--locked': selectedVehicle.transmissionOptions.length === 1 }">
              <button
                v-for="mode in selectedVehicle.transmissionOptions"
                :key="mode"
                type="button"
                class="transmission-button"
                :class="{ 'transmission-button--selected': selectedTransmission === mode }"
                @click="chooseTransmission(mode)"
              >
                <span class="transmission-button__code">{{ mode === 'manual' ? 'M' : 'A' }}</span>
                <span>
                  <strong>{{ mode === 'manual' ? 'MANUAL SEQUENTIAL' : 'REAL AUTOMATIC' }}</strong>
                  <small>{{ mode === 'manual' ? 'Q / R SHIFT · OPTIONAL CLUTCH' : 'TORQUE CONVERTER · TCM SHIFT MAP' }}</small>
                </span>
              </button>
            </div>

            <div v-if="errorMessage" class="error-message" role="alert">{{ errorMessage }}</div>
            <button class="start-button" type="button" :disabled="isLoading" @click="startDriving">
              <span>{{ isLoading ? 'LOADING VEHICLE…' : `START ${selectedModeLabel.toUpperCase()}` }}</span>
              <span class="start-button__arrow">→</span>
            </button>
          </section>
        </div>
      </template>

      <template v-else-if="activeTab === 'garage'">
        <div class="eyebrow">VEHICLE GARAGE</div>
        <h1>Your vehicles</h1>
        <p class="lead">Select a vehicle to inspect its setup before driving.</p>
        <div class="garage-list">
          <button
            v-for="vehicle in vehicles"
            :key="vehicle.id"
            type="button"
            class="garage-row"
            :class="{ 'garage-row--selected': selectedVehicleId === vehicle.id }"
            @click="chooseVehicle(vehicle)"
          >
            <span class="garage-row__icon">▰</span>
            <span class="garage-row__details">
              <strong>{{ vehicle.name }}</strong>
              <small>{{ vehicle.description }}</small>
            </span>
            <span class="garage-row__transmission">{{ vehicle.transmission }}</span>
          </button>
        </div>
        <button class="start-button start-button--secondary" type="button" @click="activeTab = 'drive'">
          BACK TO DRIVE <span class="start-button__arrow">→</span>
        </button>
      </template>

      <template v-else-if="activeTab === 'telemetry'">
        <div class="eyebrow">VIRTUAL OBD-II</div>
        <h1>Telemetry</h1>
        <p class="lead">Live vehicle signals and subsystem health.</p>
        <div class="telemetry-panel"><TelemetryDashboard :compact="false" /></div>
      </template>

      <template v-else>
        <div class="eyebrow">SYSTEM CONFIGURATION</div>
        <h1>Settings</h1>
        <p class="lead">Tune the presentation layer for your hardware.</p>
        <div class="settings-panel">
          <GraphicsSettings />
          <label class="setting-row">
            <span>
              <strong>Effective preset</strong>
              <small>Current renderer quality: {{ effectivePreset }}</small>
            </span>
            <select :value="effectivePreset" @change="setPreset(($event.target as HTMLSelectElement).value as GraphicsPreset)">
              <option value="low">Low</option>
              <option value="medium">Medium</option>
              <option value="high">High</option>
            </select>
          </label>
          <label class="setting-row setting-row--stacked">
            <span>
              <strong>Freeroam multiplayer</strong>
              <small>Connect the driving scene to the optional bounded WebSocket room server.</small>
            </span>
            <input v-model="multiplayerEnabled" type="checkbox" @change="saveMultiplayerSettings">
          </label>
          <label class="network-field">
            <span>SERVER URL</span>
            <input v-model="multiplayerServerUrl" type="url" placeholder="ws://localhost:8787" @change="saveMultiplayerSettings">
          </label>
          <label class="network-field">
            <span>ROOM</span>
            <input v-model="multiplayerRoomId" type="text" maxlength="64" pattern="[A-Za-z0-9_-]+" placeholder="freeroam" @change="saveMultiplayerSettings">
          </label>
          <button class="reset-button" type="button" @click="resetSession">Reset current session</button>
        </div>
      </template>
    </main>

    <footer class="menu-footer">
      <span>WASD DRIVE · {{ selectedTransmission === 'manual' ? 'Q/R SHIFT' : 'TCM AUTO' }}<template v-if="selectedVehicle.transmissionOptions.length > 1"> · T TRANSMISSION</template> · E IGNITION</span>
      <span>KEMUDI / TEST SESSION 01</span>
    </footer>
  </section>
</template>

<style scoped>
.main-menu {
  width: min(1180px, calc(100vw - 32px));
  max-height: calc(100vh - 48px);
  overflow: auto;
  color: #e8edf5;
  background: linear-gradient(135deg, rgba(8, 13, 20, .985), rgba(16, 25, 34, .975));
  border: 1px solid rgba(141, 182, 182, .34);
  border-radius: 7px;
  box-shadow: 0 30px 100px rgba(0, 0, 0, .62), inset 0 1px rgba(255, 255, 255, .08);
  font-family: Inter, ui-sans-serif, system-ui, sans-serif;
}

.menu-header, .menu-footer, .menu-tabs, .menu-content { padding-left: 36px; padding-right: 36px; }
.menu-header { display: flex; align-items: center; justify-content: space-between; padding-top: 28px; padding-bottom: 25px; }
.brand-lockup { display: flex; align-items: center; gap: 11px; }
.brand-mark { display: grid; width: 38px; height: 38px; place-items: center; color: #07141c; background: #79e2c5; border-radius: 4px; font-size: 21px; font-weight: 900; transform: skew(-10deg); box-shadow: 0 0 24px rgba(110, 231, 197, .18); }
.brand-name { margin: 0; letter-spacing: .24em; font-size: 15px; font-weight: 800; }
.brand-subtitle, .eyebrow, .section-heading, .build-status, .menu-footer { letter-spacing: .14em; font-size: 10px; font-weight: 700; }
.brand-subtitle { margin: 3px 0 0; color: #8290a8; }
.build-status { color: #9ba7b9; }
.status-dot { display: inline-block; width: 6px; height: 6px; margin-right: 6px; background: #6ee7c5; border-radius: 50%; box-shadow: 0 0 10px #6ee7c5; }
.menu-tabs { display: flex; gap: 4px; border-top: 1px solid rgba(153, 174, 207, .13); border-bottom: 1px solid rgba(153, 174, 207, .13); }
.menu-tab { flex: 1; padding: 15px 8px 13px; color: #8490a4; background: transparent; border: 0; border-bottom: 2px solid transparent; cursor: pointer; font-size: 12px; font-weight: 700; transition: color .2s, border-color .2s; }
.menu-tab:hover, .menu-tab--active { color: #f1f5fb; }
.menu-tab--active { border-bottom-color: #6ee7c5; }
.menu-tab__icon { margin-right: 6px; color: #6ee7c5; }
.menu-content { padding-top: 32px; padding-bottom: 34px; }
.eyebrow { color: #6ee7c5; }
h1 { margin: 7px 0 5px; color: #f6f8fc; font-size: clamp(28px, 5vw, 42px); letter-spacing: -.04em; }
h2 { margin: 7px 0 5px; color: #f6f8fc; font-size: clamp(24px, 3vw, 34px); letter-spacing: -.04em; }
.lead { margin: 0 0 24px; color: #8f9bb0; font-size: 14px; }
.drive-layout { display: grid; grid-template-columns: minmax(240px, .34fr) minmax(0, 1fr); gap: 34px; }
.drive-overview { position: relative; min-height: 520px; padding: 7px 28px 0 0; border-right: 1px solid rgba(141, 182, 182, .2); overflow: hidden; }
.drive-overview::after { content: ''; position: absolute; right: 18px; bottom: -80px; width: 230px; height: 230px; border: 1px solid rgba(110, 231, 197, .13); border-radius: 50%; box-shadow: 0 0 0 20px rgba(110, 231, 197, .025), 0 0 0 42px rgba(110, 231, 197, .02); pointer-events: none; }
.drive-overview__eyebrow { color: #6f8192; font: 700 9px/1.2 ui-monospace, SFMono-Regular, Menlo, monospace; letter-spacing: .15em; }
.drive-overview__gauge { display: flex; align-items: center; gap: 12px; margin-top: 35px; }
.drive-overview__gauge-ring { display: grid; width: 46px; height: 46px; place-items: center; color: #08141b; background: #79e2c5; border-radius: 50%; box-shadow: 0 0 0 5px rgba(121, 226, 197, .08); font-size: 20px; font-weight: 900; }
.drive-overview__gauge span:last-child { display: grid; gap: 4px; }
.drive-overview__gauge strong { color: #e7faf4; font-size: 12px; letter-spacing: .1em; }
.drive-overview__gauge small, .drive-overview__note small { color: #708196; font-size: 9px; letter-spacing: .1em; }
.drive-overview h1 { margin-top: 37px; font-size: clamp(31px, 4vw, 52px); line-height: .98; }
.drive-overview .lead { max-width: 260px; line-height: 1.55; }
.drive-readout { display: grid; gap: 1px; margin-top: 31px; border-top: 1px solid rgba(153, 174, 207, .16); border-bottom: 1px solid rgba(153, 174, 207, .16); }
.drive-readout div { display: grid; grid-template-columns: 68px 1fr; align-items: baseline; gap: 9px; padding: 12px 0; }
.drive-readout span, .drive-readout small { color: #6f8192; font-size: 9px; letter-spacing: .13em; }
.drive-readout strong { color: #e9f5f2; font-size: 12px; letter-spacing: .06em; }
.drive-readout small { grid-column: 2; margin-top: -7px; }
.drive-overview__vehicle-line { display: flex; align-items: center; gap: 8px; margin-top: 25px; color: #79e2c5; font: 700 9px ui-monospace, SFMono-Regular, Menlo, monospace; letter-spacing: .12em; }
.drive-overview__vehicle-line i { display: block; width: 19px; height: 2px; background: #79e2c5; box-shadow: 7px 4px 0 rgba(121, 226, 197, .35); }
.drive-overview__note { position: absolute; right: 28px; bottom: 18px; left: 0; display: grid; gap: 5px; padding-top: 16px; border-top: 1px solid rgba(153, 174, 207, .13); }
.drive-overview__note span { color: #6f8192; font-size: 9px; letter-spacing: .13em; }
.drive-overview__note strong { color: #dceee9; font-size: 10px; letter-spacing: .06em; }
.drive-config { min-width: 0; }
.section-heading--compact { margin-top: 26px; }
.mode-grid, .vehicle-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
.mode-card, .vehicle-card, .garage-row { color: inherit; text-align: left; cursor: pointer; }
.mode-card { position: relative; min-height: 108px; padding: 17px; background: linear-gradient(120deg, rgba(255, 255, 255, .05), rgba(255, 255, 255, .018)); border: 1px solid rgba(153, 174, 207, .16); border-radius: 4px; }
.mode-card:hover, .mode-card--selected, .vehicle-card:hover, .vehicle-card--selected { border-color: rgba(110, 231, 197, .8); background: rgba(110, 231, 197, .09); }
.mode-card__number { display: block; margin-bottom: 17px; color: #65738b; font-size: 10px; font-weight: 800; letter-spacing: .15em; }
.mode-card__title, .vehicle-card__name { display: block; color: #f0f4fa; font-size: 15px; font-weight: 750; }
.mode-card__copy { display: block; max-width: 250px; margin-top: 5px; color: #8996aa; font-size: 11px; line-height: 1.4; }
.section-heading { display: flex; justify-content: space-between; margin: 27px 0 10px; color: #9ba7b9; }
.section-heading--map { margin-top: 22px; }
.section-heading--transmission { margin-top: 22px; }
.section-heading__count { color: #5e6c81; }
.vehicle-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
.vehicle-card { min-height: 100px; padding: 14px; background: rgba(255, 255, 255, .035); border: 1px solid rgba(153, 174, 207, .16); border-radius: 4px; }
.vehicle-card__silhouette { display: block; margin-bottom: 13px; color: #6ee7c5; font-size: 20px; transform: scaleX(1.4); transform-origin: left; }
.vehicle-card__drive { display: block; margin-top: 5px; color: #748197; font-size: 10px; }
.map-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
.map-card { min-height: 150px; padding: 0; overflow: hidden; color: inherit; text-align: left; background: rgba(255, 255, 255, .035); border: 1px solid rgba(153, 174, 207, .16); border-radius: 4px; cursor: pointer; }
.map-card:hover, .map-card--selected { border-color: rgba(110, 231, 197, .8); background: rgba(110, 231, 197, .09); }
.map-card__preview { position: relative; display: block; height: 62px; overflow: hidden; background-color: #3b4650; background-image: linear-gradient(135deg, rgba(255,255,255,.18) 25%, transparent 25%, transparent 50%, rgba(255,255,255,.1) 50%, rgba(255,255,255,.1) 75%, transparent 75%); background-size: 18px 18px; }
.map-card__preview::before { content: ''; position: absolute; top: -20px; bottom: -20px; left: 50%; width: 30%; transform: translateX(-50%) rotate(4deg); background: #1b2329; box-shadow: 0 0 0 2px rgba(233, 217, 154, .16); }
.map-card__preview i, .map-card__preview b { position: absolute; z-index: 1; display: block; width: 3px; height: 8px; background: #e9d99a; }
.map-card__preview i { left: 48%; bottom: 7px; }.map-card__preview b { right: 48%; top: 7px; }
.map-card__preview--bumpy { background-color: #46534a; background-size: 9px 12px; }.map-card__preview--bumpy::before { transform: translateX(-50%) rotate(-5deg); background: #202a28; }
.map-card__preview--offroad { background-color: #806347; background-image: radial-gradient(rgba(45,25,14,.28) 1px, transparent 1px); background-size: 9px 9px; }.map-card__preview--offroad::before { width: 42%; transform: translateX(-50%) rotate(2deg); background: #5b4631; }
.map-card__name, .map-card__surface, .map-card__copy { display: block; margin-left: 12px; margin-right: 12px; }
.map-card__name { margin-top: 10px; color: #f0f4fa; font-size: 13px; font-weight: 750; }
.map-card__surface { margin-top: 3px; color: #6ee7c5; font-size: 9px; letter-spacing: .12em; }
.map-card__copy { margin-top: 6px; margin-bottom: 12px; color: #8996aa; font-size: 10px; line-height: 1.35; }
.transmission-choice { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
.transmission-choice--locked { grid-template-columns: 1fr; }
.transmission-button { display: flex; align-items: center; gap: 12px; min-height: 66px; padding: 12px; color: #e8edf5; text-align: left; background: rgba(255,255,255,.035); border: 1px solid rgba(153,174,207,.16); border-radius: 4px; cursor: pointer; }
.transmission-button:hover, .transmission-button--selected { color: #f4fffb; background: rgba(110,231,197,.09); border-color: rgba(110,231,197,.8); }
.transmission-button__code { display: grid; width: 30px; height: 30px; place-items: center; color: #07141c; background: #6ee7c5; border-radius: 5px; font-size: 15px; font-weight: 900; }
.transmission-button:not(.transmission-button--selected) .transmission-button__code { color: #8e9bae; background: rgba(255,255,255,.08); }
.transmission-button span:last-child { display: grid; gap: 4px; }
.transmission-button strong { font-size: 11px; letter-spacing: .05em; }
.transmission-button small { color: #8490a4; font-size: 9px; letter-spacing: .04em; }
.error-message { margin-top: 14px; padding: 10px 12px; color: #ffb4b4; background: rgba(160, 35, 55, .2); border: 1px solid rgba(255, 120, 130, .4); border-radius: 8px; font-size: 12px; }
.start-button { display: flex; align-items: center; justify-content: space-between; width: 100%; margin-top: 18px; padding: 16px 17px; color: #07141c; background: #79e2c5; border: 0; border-radius: 4px; cursor: pointer; font-size: 12px; font-weight: 900; letter-spacing: .14em; transition: background .2s, transform .2s; }
.start-button:hover { background: #9af5de; transform: translateY(-1px); }
.start-button:disabled { cursor: wait; opacity: .65; transform: none; }
.start-button--secondary { color: #dce8f1; background: rgba(255, 255, 255, .08); }
.start-button__arrow { font-size: 20px; letter-spacing: 0; }
.garage-list { display: grid; gap: 9px; }
.garage-row { display: flex; align-items: center; gap: 14px; padding: 15px; color: #e8edf5; background: rgba(255,255,255,.035); border: 1px solid rgba(153,174,207,.16); border-radius: 10px; }
.garage-row--selected { border-color: rgba(110,231,197,.8); background: rgba(110,231,197,.09); }
.garage-row__icon { color: #6ee7c5; font-size: 20px; }
.garage-row__details { display: grid; flex: 1; gap: 4px; }
.garage-row__details small, .setting-row small { color: #8490a4; font-size: 11px; }
.garage-row__transmission { color: #6ee7c5; font-size: 10px; text-transform: uppercase; }
.telemetry-panel, .settings-panel { padding: 18px; background: rgba(0,0,0,.18); border: 1px solid rgba(153,174,207,.14); border-radius: 11px; }
.settings-panel :deep(.graphics-settings) { padding: 0 0 17px; background: transparent; border-radius: 0; border-bottom: 1px solid rgba(153,174,207,.14); }
.setting-row { display: flex; align-items: center; justify-content: space-between; gap: 20px; padding: 17px 0; color: #e8edf5; }
.setting-row span { display: grid; gap: 5px; }
.setting-row--stacked { align-items: flex-start; }
.network-field { display: grid; gap: 7px; margin-top: 12px; color: #7f8ca2; font-size: 9px; font-weight: 800; letter-spacing: .12em; }
.network-field input { width: 100%; padding: 10px 11px; color: #e8edf5; background: rgba(255,255,255,.05); border: 1px solid rgba(153,174,207,.2); border-radius: 6px; outline: none; font: 12px ui-monospace, SFMono-Regular, Menlo, monospace; letter-spacing: 0; }
.network-field input:focus { border-color: #6ee7c5; }
.setting-row select { padding: 8px; color: #e8edf5; background: #1b2434; border: 1px solid #46546c; border-radius: 6px; }
.reset-button { padding: 8px 11px; color: #9ba7b9; background: transparent; border: 1px solid #46546c; border-radius: 6px; cursor: pointer; font-size: 11px; }
.menu-footer { display: flex; justify-content: space-between; padding-top: 14px; padding-bottom: 19px; color: #56647a; border-top: 1px solid rgba(153,174,207,.13); }
@media (max-width: 820px) { .drive-layout { grid-template-columns: 1fr; gap: 26px; } .drive-overview { min-height: 0; padding: 0 0 24px; border-right: 0; border-bottom: 1px solid rgba(141, 182, 182, .2); } .drive-overview::after { display: none; } .drive-overview__gauge { margin-top: 22px; } .drive-overview h1 { margin-top: 24px; } .drive-overview__note { position: static; margin-top: 22px; } }
@media (max-width: 600px) { .menu-header, .menu-footer, .menu-tabs, .menu-content { padding-left: 18px; padding-right: 18px; } .build-status { display: none; } .menu-tab { font-size: 10px; } .menu-tab__icon { display: block; margin: 0 0 4px; } .vehicle-grid, .map-grid, .transmission-choice { grid-template-columns: 1fr; } .menu-footer { font-size: 8px; gap: 12px; flex-direction: column; } }
</style>
