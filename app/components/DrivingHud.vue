<script setup lang="ts">
import { computed } from 'vue'
import { ref } from 'vue'
import type { CameraMode } from '~/composables/useCameraSystem'
import type { InputState } from '~/composables/useInput'
import type { IgnitionState } from '~/stores/vehicleSession'

const props = defineProps<{
  vehicleName: string
  modeLabel: string
  mapLabel: string
  speed: number
  rpm: number
  gear: number
  cameraMode: CameraMode
  ignitionState: IgnitionState
  input: Readonly<InputState>
  obd: { coolant: number; oilPressure: number; fuel: number; grip: number; damage: number; abs: boolean; tc: boolean; fcw: boolean; aeb: boolean }
}>()

const emit = defineEmits<{
  menu: []
  camera: []
}>()

const gearLabel = computed(() => {
  if (props.gear < 0) return 'R'
  if (props.gear === 0) return 'N'
  return String(props.gear)
})

const speedDisplay = computed(() => Math.max(0, Math.round(props.speed)))
const rpmPercent = computed(() => Math.min(100, Math.max(0, (props.rpm / 9000) * 100)))
const obdVisible = ref(true)
</script>

<template>
  <div class="driving-hud" aria-label="Driving interface">
    <div class="driving-hud__top">
      <div class="vehicle-badge">
        <span class="vehicle-badge__dot" />
        <span><strong>{{ vehicleName }}</strong><small>{{ modeLabel }} · {{ mapLabel }}</small></span>
      </div>
      <div class="hud-actions">
        <button type="button" @click="emit('camera')">CAM {{ cameraMode.toUpperCase() }}</button>
        <button v-if="cameraMode !== 'interior'" type="button" @click="obdVisible = !obdVisible">OBD {{ obdVisible ? '×' : '+' }}</button>
        <button type="button" @click="emit('menu')">MENU <kbd>Esc</kbd></button>
      </div>
    </div>

    <aside v-if="cameraMode !== 'interior' && obdVisible" class="obd-panel" aria-label="Vehicle OBD information">
      <div class="obd-panel__header"><span>LIVE VEHICLE OBD</span><button type="button" aria-label="Dismiss OBD panel" @click="obdVisible = false">×</button></div>
      <div class="obd-status"><i :class="`obd-status__dot obd-status__dot--${ignitionState}`" /> IGNITION: <strong>{{ ignitionState.toUpperCase() }}</strong></div>
      <div class="obd-grid">
        <div><small>ENGINE RPM</small><strong>{{ Math.round(rpm) }}</strong><em>rpm</em></div>
        <div><small>VEHICLE SPEED</small><strong>{{ speedDisplay }}</strong><em>km/h</em></div>
        <div><small>GEAR</small><strong>{{ gearLabel }}</strong><em>current</em></div>
        <div><small>SURFACE</small><strong>{{ mapLabel }}</strong><em>active map</em></div>
        <div><small>COOLANT</small><strong>{{ Math.round(obd.coolant) }}</strong><em>°C</em></div>
        <div><small>OIL PRESSURE</small><strong>{{ Math.round(obd.oilPressure) }}</strong><em>kPa</em></div>
        <div><small>FUEL</small><strong>{{ Math.round(obd.fuel) }}</strong><em>%</em></div>
        <div><small>HEALTH</small><strong>{{ Math.round((1 - obd.damage) * 100) }}</strong><em>%</em></div>
      </div>
      <div class="obd-assists"><span :class="{ 'obd-assists__active': obd.abs }">ABS {{ obd.abs ? 'ACTIVE' : 'READY' }}</span><span :class="{ 'obd-assists__active': obd.tc }">TCS {{ obd.tc ? 'ACTIVE' : 'READY' }}</span><span :class="{ 'obd-assists__active': obd.fcw }">FCW {{ obd.fcw ? 'WARN' : 'READY' }}</span><span :class="{ 'obd-assists__active': obd.aeb }">AEB {{ obd.aeb ? 'BRAKE' : 'READY' }}</span><span>GRIP {{ Math.round(obd.grip * 100) }}%</span></div>
      <div class="obd-hint" v-if="ignitionState !== 'running'">Press <kbd>E</kbd> for {{ ignitionState === 'off' ? 'accessory' : 'engine start' }}</div>
    </aside>

    <div class="driving-hud__center">
      <span class="reticle reticle--left" />
      <span class="reticle reticle--right" />
      <span class="reticle reticle--center" />
    </div>

    <div class="driving-hud__bottom">
      <div class="control-card">
        <div class="control-card__title">CONTROL INPUT</div>
        <div class="input-row"><span>STEER</span><div class="input-track"><i class="input-fill input-fill--steer" :style="{ width: `${Math.abs(input.steering) * 50}%`, marginLeft: `${input.steering >= 0 ? 50 : 50 - Math.abs(input.steering) * 50}%` }" /></div></div>
        <div class="input-row"><span>THROTTLE</span><div class="input-track"><i class="input-fill input-fill--throttle" :style="{ width: `${input.throttle * 100}%` }" /></div></div>
        <div class="input-row"><span>BRAKE</span><div class="input-track"><i class="input-fill input-fill--brake" :style="{ width: `${input.brake * 100}%` }" /></div></div>
        <div class="control-hints"><kbd>W</kbd> throttle <kbd>S</kbd> brake <kbd>A/D</kbd> steer <kbd>E</kbd> ignition</div>
      </div>

      <div v-if="cameraMode !== 'interior'" class="speed-card">
        <div class="speed-card__gear">{{ gearLabel }}</div>
        <div class="speed-card__speed">{{ speedDisplay }}<small>KM/H</small></div>
        <div class="rpm-track"><i :style="{ width: `${rpmPercent}%` }" /></div>
        <div class="speed-card__rpm">{{ Math.round(rpm) }} RPM</div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.driving-hud { position: absolute; inset: 0; z-index: 4; pointer-events: none; color: #eef4f8; font-family: Inter, ui-sans-serif, system-ui, sans-serif; }
.driving-hud__top, .driving-hud__bottom { position: absolute; left: 20px; right: 20px; display: flex; justify-content: space-between; pointer-events: auto; }
.driving-hud__top { top: 20px; }
.vehicle-badge, .control-card, .speed-card { background: rgba(8, 14, 23, .78); border: 1px solid rgba(153, 174, 207, .24); border-radius: 9px; box-shadow: 0 8px 30px rgba(0,0,0,.2); backdrop-filter: blur(9px); }
.vehicle-badge { display: flex; align-items: center; gap: 9px; padding: 10px 12px; }
.vehicle-badge__dot { width: 7px; height: 7px; background: #6ee7c5; border-radius: 50%; box-shadow: 0 0 10px #6ee7c5; }
.vehicle-badge span:last-child { display: grid; gap: 3px; }
.vehicle-badge strong { font-size: 12px; }
.vehicle-badge small, .control-card__title, .speed-card__rpm { color: #8a99ac; font-size: 9px; letter-spacing: .14em; text-transform: uppercase; }
.hud-actions { display: flex; gap: 7px; }
.hud-actions button { align-self: flex-start; padding: 9px 11px; color: #dce8f1; background: rgba(8,14,23,.76); border: 1px solid rgba(153,174,207,.24); border-radius: 6px; cursor: pointer; font-size: 9px; font-weight: 800; letter-spacing: .1em; }
.hud-actions button:hover { color: #07141c; background: #6ee7c5; }
kbd { padding: 2px 4px; color: #6ee7c5; background: rgba(110,231,197,.1); border-radius: 3px; font-size: 9px; }
.driving-hud__center { position: absolute; top: 50%; left: 50%; width: 24px; height: 24px; transform: translate(-50%, -50%); opacity: .45; }
.reticle { position: absolute; display: block; background: #d8f9ee; }
.reticle--center { top: 11px; left: 11px; width: 2px; height: 2px; border-radius: 50%; }
.reticle--left, .reticle--right { top: 11px; width: 6px; height: 1px; }
.reticle--left { left: 0; }.reticle--right { right: 0; }
.driving-hud__bottom { bottom: 20px; align-items: end; }
.obd-panel { position: absolute; top: 96px; left: 20px; width: 242px; padding: 12px; pointer-events: auto; background: rgba(8,14,23,.84); border: 1px solid rgba(153,174,207,.24); border-radius: 9px; box-shadow: 0 8px 30px rgba(0,0,0,.2); backdrop-filter: blur(9px); }
.obd-panel__header { display: flex; justify-content: space-between; align-items: center; color: #6ee7c5; font-size: 9px; font-weight: 850; letter-spacing: .14em; }
.obd-panel__header button { color: #9aa8b8; background: transparent; border: 0; cursor: pointer; font-size: 18px; line-height: .8; }
.obd-status { display: flex; align-items: center; gap: 5px; margin-top: 10px; color: #8996aa; font-size: 9px; letter-spacing: .09em; }.obd-status strong { color: #e9f4f4; }.obd-status__dot { width: 6px; height: 6px; border-radius: 50%; }.obd-status__dot--off { background: #7c8797; }.obd-status__dot--accessory { background: #f5d66d; box-shadow: 0 0 8px #f5d66d; }.obd-status__dot--running { background: #6ee7c5; box-shadow: 0 0 8px #6ee7c5; }
.obd-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; margin-top: 12px; }.obd-grid div { padding: 7px; background: rgba(255,255,255,.045); border-radius: 5px; }.obd-grid small, .obd-grid em { display: block; color: #718097; font-size: 8px; font-style: normal; letter-spacing: .08em; }.obd-grid strong { display: inline-block; margin-top: 4px; color: #edf5f5; font-size: 13px; }.obd-grid em { display: inline; margin-left: 3px; font-size: 8px; }
.obd-assists { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 9px; color: #718097; font-size: 8px; letter-spacing: .08em; }.obd-assists span { padding: 4px 5px; background: rgba(255,255,255,.04); border-radius: 3px; }.obd-assists__active { color: #f5d66d; }
.obd-hint { margin-top: 11px; color: #9aa8b8; font-size: 9px; }.obd-hint kbd { margin-left: 3px; }
.control-card { width: 245px; padding: 12px; }
.control-card__title { margin-bottom: 9px; color: #6ee7c5; }
.input-row { display: grid; grid-template-columns: 55px 1fr; align-items: center; gap: 8px; margin: 6px 0; color: #8996aa; font-size: 9px; font-weight: 800; }
.input-track, .rpm-track { position: relative; overflow: hidden; height: 4px; background: rgba(255,255,255,.12); border-radius: 3px; }
.input-fill, .rpm-track i { display: block; height: 100%; border-radius: inherit; }.input-fill--steer { background: #6ee7c5; }.input-fill--throttle { background: #8ad6ff; }.input-fill--brake { background: #ff8b91; }.rpm-track i { background: linear-gradient(90deg,#6ee7c5,#f5d66d,#ff858d); }
.control-hints { margin-top: 11px; color: #65738b; font-size: 9px; }
.control-hints kbd { margin-left: 4px; }.control-hints kbd:first-child { margin-left: 0; }
.speed-card { min-width: 190px; padding: 12px 14px; text-align: right; }
.speed-card__gear { float: left; color: #6ee7c5; font-size: 29px; font-weight: 850; line-height: 1; }
.speed-card__speed { color: #f4f8fb; font-size: 39px; font-weight: 800; letter-spacing: -.06em; line-height: .95; }.speed-card__speed small { margin-left: 5px; color: #8a99ac; font-size: 9px; letter-spacing: .12em; }
.rpm-track { clear: both; margin-top: 13px; }.speed-card__rpm { margin-top: 6px; }
@media (max-width: 600px) { .driving-hud__top, .driving-hud__bottom { left: 10px; right: 10px; }.obd-panel { top: 88px; left: 10px; width: 205px; }.control-card { width: 180px; }.control-hints { display: none; }.speed-card { min-width: 145px; }.speed-card__speed { font-size: 31px; }.hud-actions button:first-child { display: none; } }
</style>
