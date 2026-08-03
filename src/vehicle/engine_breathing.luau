--[[
	Engine Breathing Simulation.
	Pure Luau, no Roblox API dependencies.
	Volumetric efficiency, exhaust backpressure, altitude, accessories.
	Ambient air effects on engine performance.
]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")

local EngineBreathing = {}

-- ============================================================
-- Config
-- ============================================================

function EngineBreathing.newConfig(overrides)
	assert(type(overrides) == "table", "engine breathing parameters must be authored by the vehicle asset")
	return overrides
end

-- ============================================================
-- State
-- ============================================================

function EngineBreathing.newState()
	return {
		current_ve = 0.8,  -- current volumetric efficiency
		intake_air_temp = 25.0,  -- °C
		intake_air_density = 1.204,  -- kg/m³
		exhaust_pressure = 101325.0,  -- Pa (atmospheric base)
		altitude = 0.0,  -- meters above sea level
		ambient_temp = 25.0,  -- °C
		accessory_load = 0.0,  -- total accessory torque demand
	}
end

-- ============================================================
-- Internal helpers
-- ============================================================

local function interpolate_ve_curve(curve, rpm)
	if not curve or #curve == 0 then return 0.8 end

	-- Clamp RPM outside the curve range
	if rpm <= curve[1][1] then return curve[1][2] end
	if rpm >= curve[#curve][1] then return curve[#curve][2] end

	-- Find the two bracketing points and lerp
	for i = 1, #curve - 1 do
		local r0, v0 = curve[i][1], curve[i][2]
		local r1, v1 = curve[i + 1][1], curve[i + 1][2]
		if rpm >= r0 and rpm <= r1 then
			local t = (rpm - r0) / (r1 - r0)
			return v0 + t * (v1 - v0)
		end
	end

	return 0.8
end

local function atmospheric_pressure(altitude_m)
	-- Barometric formula approximation
	-- P = 101325 × (1 - 0.0065 × h / 288.15)^5.2561
	local base = 101325.0
	local t = 1.0 - 0.0065 * altitude_m / 288.15
	if t <= 0 then return 2000.0 end -- floor to prevent zero/negative
	return base * t ^ 5.2561
end

local function celsius_to_kelvin(c)
	return c + 273.15
end

-- ============================================================
-- Public API
-- ============================================================

function EngineBreathing.update(state, config, engine_rpm, throttle, dt)
	-- Guard: skip if vehicle asset provides no breathing config
	if not config or not config.ve_curve then return end

	-- NaN guards on inputs
	engine_rpm = Math.finite_or_zero(engine_rpm)
	throttle = Math.finite_or_zero(throttle)

	local rpm = math.max(engine_rpm, 0)
	local th = Math.clamp(throttle, 0, 1)

	-- VE from curve interpolation at current RPM
	local ve = interpolate_ve_curve(config.ve_curve, rpm)

	-- Exhaust backpressure: P_exhaust = P_atm + coeff × RPM² × th
	local p_atm = atmospheric_pressure(state.altitude)
	local exhaust_backpressure = p_atm + config.exhaust_backpressure_coeff * rpm * rpm * th
	state.exhaust_pressure = Math.finite_or_zero(exhaust_backpressure)

	-- VE reduction from backpressure
	-- backpressure_ratio = (P_exhaust - P_atm) / P_atm
	local bp_ratio = (state.exhaust_pressure - p_atm) / p_atm
	bp_ratio = math.max(bp_ratio, 0) -- no negative backpressure
	ve = ve * (1.0 - config.exhaust_pressure_penalty * bp_ratio)
	ve = Math.clamp(ve, 0, 1)

	-- Intake temperature
	-- T_intake = T_ambient + intake_temp_offset × (1 - intercooler_effectiveness × throttle)
	local intake_temp = state.ambient_temp
		+ config.intake_temp_offset * (1.0 - config.intercooler_effectiveness * th)
	state.intake_air_temp = Math.finite_or_zero(intake_temp)

	-- Air density with altitude and temperature
	-- ρ = base_density × (1 - altitude_decay × altitude/1000) × (T_base / T_intake_K)
	local t_base_K = celsius_to_kelvin(20.0)  -- 20°C standard reference
	local t_intake_K = celsius_to_kelvin(state.intake_air_temp)
	local alt_factor = 1.0 - config.altitude_decay_per_km * state.altitude / 1000.0
	alt_factor = math.max(alt_factor, 0) -- no negative density
	local temp_ratio = t_base_K / t_intake_K
	local air_density = config.base_air_density * alt_factor * temp_ratio
	state.intake_air_density = Math.finite_or_zero(air_density)
	state.current_ve = ve

	-- Accessory load
	local alternator = math.min(config.alternator_coeff * rpm, config.alternator_max_torque)
	local ac = if config.ac_enabled then config.ac_compressor_torque else 0.0
	state.accessory_load = Math.finite_or_zero(alternator + ac)

	return state
end

function EngineBreathing.getEffectiveTorque(state, config, base_torque)
	-- Guard: if update was skipped (no config), return unmodified torque
	if not state or not config then return base_torque end
	local density_ratio = state.intake_air_density or config.base_air_density or 1
	local effective_ve = state.current_ve or 1
	local accessory_load = state.accessory_load or 0
	local effective = base_torque * effective_ve * (density_ratio / math.max(config.base_air_density or 1, 1e-6))
	effective = effective - accessory_load
	return math.max(effective, 0.0)
end

function EngineBreathing.getAirDensity(state)
	return state.intake_air_density
end

function EngineBreathing.telemetry(state)
	return {
		ve = state.current_ve,
		intake_temp = state.intake_air_temp,
		exhaust_pressure = state.exhaust_pressure,
		air_density = state.intake_air_density,
		accessory_load = state.accessory_load,
	}
end

return EngineBreathing
