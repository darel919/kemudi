--[[
	Brake thermal model — heat generation, convective cooling, fade, fluid pressure.
	Simplified disc-brake simulation for soft-body vehicle physics.
	Pure Luau, no Roblox API dependencies.

	ALL CONFIG MUST BE PROVIDED BY VEHICLE ASSET.
	No vehicle-specific defaults in engine code.
--]]

local Brakes = {}

-- ============================================================
-- NaN / infinite guard
-- ============================================================

local function safeFinite(v)
	if v == nil then return 0 end
	if v ~= v then return 0 end -- NaN
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

-- nil/NaN/infinity guard — returns fallback when value is missing or invalid
local function nan_safe(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end -- NaN
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

-- ============================================================
-- Factory functions — ALL REQUIRED FROM VEHICLE ASSET
-- ============================================================

function Brakes.newState()
	return {
		disc_temp = 25.0,
		pad_temp = 25.0,
		fluid_temp = 25.0,
		fade_factor = 1.0,
		peak_temp_seen = 25.0,
		total_energy_dissipated = 0.0,
	}
end

function Brakes.newConfig(overrides)
	assert(type(overrides) == "table", "brake parameters must be authored by the vehicle asset")
	for _, name in ipairs({
		"bias_front", "disc_mass", "disc_specific_heat", "disc_area", "convective_coeff",
		"pad_friction_nominal", "fluid_boil_temp", "fade_onset_temp", "fade_critical_temp",
		"max_disc_temp", "brake_torque_max", "contact_patch_radius",
		"airflow_speed_reference", "fluid_fade_end_temp",
	}) do
		assert(type(overrides[name]) == "number", "missing authored brake parameter " .. name)
	end
	-- Engine code provides NO vehicle-specific defaults.
	-- All values MUST come from vehicle asset.
	-- If missing, engine behaves as "no effect" (nil/0) or fails loudly.
	return {
		-- Front/rear brake bias (0-1)
		bias_front = overrides.bias_front,

		-- Disc properties
		disc_mass = overrides.disc_mass,
		disc_specific_heat = overrides.disc_specific_heat,
		disc_area = overrides.disc_area,

		-- Cooling
		convective_coeff = overrides.convective_coeff,

		-- Friction
		pad_friction_nominal = overrides.pad_friction_nominal,

		-- Fluid
		fluid_boil_temp = overrides.fluid_boil_temp,

		-- Fade
		fade_onset_temp = overrides.fade_onset_temp,
		fade_critical_temp = overrides.fade_critical_temp,
		max_disc_temp = overrides.max_disc_temp,

		-- Torque
		brake_torque_max = overrides.brake_torque_max,
		contact_patch_radius = overrides.contact_patch_radius,
		airflow_speed_reference = overrides.airflow_speed_reference,
		fluid_fade_end_temp = overrides.fluid_fade_end_temp,
	}
end

-- ============================================================
-- Fade factor calculation
-- ============================================================

local function computeFadeFactor(disc_temp, fluid_temp, cfg)
	-- If vehicle asset doesn't provide fade temperatures, no fade effect
	if not cfg.fade_onset_temp or not cfg.fade_critical_temp then
		return 1.0
	end

	local fade = 1.0
	local onset = cfg.fade_onset_temp
	local critical = cfg.fade_critical_temp

	if onset >= critical then return 1.0 end -- invalid range, no fade

	if disc_temp > onset then
		local t = clamp((disc_temp - onset) / (critical - onset), 0, 1)
		fade = 1.0 - t * 0.7 -- 1.0 → 0.3
	end

	-- Severe fade above critical
	local max_temp = nan_safe(cfg.max_disc_temp, critical * 1.25)
	if disc_temp >= critical then
		fade = clamp(fade, 0.15, 0.3)
	end

	-- Fluid fade
	if cfg.fluid_boil_temp and fluid_temp > cfg.fluid_boil_temp then
		fade = fade * 0.3
	end

	return fade
end

-- ============================================================
-- Thermal update (mutates state in-place)
-- ============================================================

function Brakes.update(state, config, speed, ambient_temp, dt)
	if state[1] then
		for _, wheelState in ipairs(state) do
			Brakes.update(wheelState, config, speed, ambient_temp, dt)
		end
		return state
	end
	-- Guard: if vehicle asset provides no brake thermal config, skip thermal update
	if not config or not config.disc_mass then return end

	-- Ensure state fields exist (safe for old state objects)
	if state.disc_temp == nil then state.disc_temp = 25.0 end
	if state.pad_temp == nil then state.pad_temp = 25.0 end
	if state.fluid_temp == nil then state.fluid_temp = 25.0 end

	-- NaN guards
	dt = nan_safe(dt, 1/60)
	speed = nan_safe(speed, 0)
	ambient_temp = nan_safe(ambient_temp, 25)

	local discMass = nan_safe(config.disc_mass, 1)
	local discC = nan_safe(config.disc_specific_heat, 1)
	local discArea = nan_safe(config.disc_area, 1)
	local h = nan_safe(config.convective_coeff, 1)
	local airflowReference = assert(config.airflow_speed_reference, "brake airflow speed reference was not authored")
	local airflow = 0.5 + 0.5 * clamp(math.abs(speed) / math.max(airflowReference, 1e-6), 0, 1)
	h *= airflow
	local T_amb = ambient_temp

	-- Energy input: brake power = torque * angular_velocity
	-- We don't have angular velocity here, so estimate from speed and brake pressure
	-- Caller must provide brake_torque_applied or we estimate
	local brakeTorque = state._last_brake_torque or 0
	local radius = state._tire_radius or config.contact_patch_radius
	assert(radius, "brake state tire radius was not initialized")
	local angularSpeed = speed / math.max(radius, 1e-6)
	local power = brakeTorque * angularSpeed -- Watts
	local heatToDisc = power * dt
	state.disc_temp = state.disc_temp + heatToDisc / math.max(discMass * discC, 1)

	-- Disc -> pad conduction (simplified: 30% of disc heat flows to pad per second)
	local discToPad = (state.disc_temp - state.pad_temp) * 0.3 * dt
	state.pad_temp = state.pad_temp + discToPad

	-- Pad -> fluid (10% of pad heat flows to fluid per second)
	local padToFluid = (state.pad_temp - state.fluid_temp) * 0.1 * dt
	state.fluid_temp = state.fluid_temp + padToFluid

	-- Convective cooling: Q = h * A * (T - T_amb) * dt
	local discCool = h * discArea * (state.disc_temp - T_amb) * dt
	state.disc_temp = state.disc_temp - discCool / math.max(discMass * discC, 1)

	local padCool = h * discArea * 0.5 * (state.pad_temp - T_amb) * dt -- pads have ~half area
	state.pad_temp = state.pad_temp - padCool / math.max(discMass * discC * 0.3, 1)

	local fluidCool = h * discArea * 0.1 * (state.fluid_temp - T_amb) * dt -- fluid has little direct airflow
	state.fluid_temp = state.fluid_temp - fluidCool / math.max(discMass * discC * 0.1, 1)

	-- Clamp to physical limits
	state.disc_temp = clamp(state.disc_temp, T_amb, nan_safe(config.max_disc_temp, 1000))
	state.pad_temp = clamp(state.pad_temp, T_amb, nan_safe(config.max_disc_temp, 1000))
	state.fluid_temp = clamp(state.fluid_temp, T_amb, nan_safe(config.fluid_boil_temp + 100, 400))

	-- Update fade factor
	state.fade_factor = computeFadeFactor(state.disc_temp, state.fluid_temp, config)

	-- Track peak temp
	state.peak_temp_seen = math.max(state.peak_temp_seen, state.disc_temp)
	state.total_energy_dissipated = state.total_energy_dissipated + power * dt

	return state
end

-- Set the brake torque applied this frame (for thermal calculation)
function Brakes.setBrakeTorque(state, torque)
	state._last_brake_torque = nan_safe(torque, 0)
end

-- Returns effective pressure multiplier from fade (0-1)
function Brakes.getEffectivePressure(state, config)
	if not state then return 0 end
	local fade = nan_safe(state.fade_factor, 1.0)
	if config and config.fluid_boil_temp and config.fluid_fade_end_temp and state.fluid_temp > config.fluid_boil_temp then
		fade *= clamp((config.fluid_fade_end_temp - state.fluid_temp) / (config.fluid_fade_end_temp - config.fluid_boil_temp), 0, 1)
	end
	return fade
end

-- Returns brake torque for given pressure (0-1)
function Brakes.computeBrakeTorque(state, config, pressure)
	local maxTorque = nan_safe(config.brake_torque_max, 0)
	return pressure * maxTorque
end

function Brakes.calculateForces(states, config, pressure, tire_radius, wheel_speeds)
	local forces = {}
	for i, state in ipairs(states) do
		local bias = if i <= 2 then config.bias_front else 1 - config.bias_front
		local fade = Brakes.getEffectivePressure(state, config)
		forces[i] = math.clamp(pressure, 0, 1) * bias * config.brake_torque_max * fade
	end
	return forces
end

function Brakes.telemetry(state)
	return {
		disc_temp = state.disc_temp,
		pad_temp = state.pad_temp,
		fluid_temp = state.fluid_temp,
		fade_factor = state.fade_factor,
		peak_temp = state.peak_temp_seen,
		energy_dissipated = state.total_energy_dissipated,
	}
end

return Brakes