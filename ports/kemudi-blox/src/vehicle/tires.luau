--[[
	Tire model — Pacejka magic formula, thermal, wear, failure.
	Ported from crates/kemudi-engine/src/tires.rs.
	Pure Luau, no Roblox API dependencies.
--]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")

local Tires = {}

-- ============================================================
-- Compound lookup tables
-- ============================================================
-- { base_grip, temp_sensitivity, wear_rate, opt_temp_min, opt_temp_max }
local COMPOUND_DATA = {
	Sport    = { 0.95, 1.2, 1.5, 80, 120 },
	Street   = { 0.85, 1.0, 1.0, 60, 100 },
	Offroad  = { 0.70, 0.8, 0.8, 50, 90 },
	Mud      = { 0.50, 0.6, 0.7, 40, 80 },
	Snow     = { 0.40, 0.7, 0.6, 30, 70 },
}

local REFERENCE_TIRE_LOAD = 4000.0

-- ============================================================
-- Factory functions
-- ============================================================

function Tires.newState(compound)
	compound = compound or "Street"
	return {
		compound = compound,
		temperature = 20.0,
		pressure = 220.0,
		nominal_pressure = 220.0,
		wear = 0.0,
		grip_factor = (COMPOUND_DATA[compound] or COMPOUND_DATA.Street)[1],
		is_flat = false,
		has_blister = false,
		has_puncture = false,
		cord_exposed = false,
		blowout_risk = 0.0,
		center_wear = 0.0,
		shoulder_wear = 0.0,
	}
end

function Tires.newThermalParams(overrides)
	local p = {
		heat_generation_rate = 5.0,
		heat_dissipation_rate = 50.0,
		thermal_mass = 8.0,
	}
	if overrides then for k, v in overrides do p[k] = v end end
	return p
end

function Tires.newPacejkaParams(overrides)
	local p = {
		longitudinal_b = 10.0, longitudinal_c = 1.65, longitudinal_d = 1.0, longitudinal_e = 0.97,
		lateral_b = 10.0, lateral_c = 1.9, lateral_d = 1.0, lateral_e = 0.97,
	}
	if overrides then for k, v in overrides do p[k] = v end end
	return p
end

-- ============================================================
-- Private helpers
-- ============================================================

local function safeFinite(v)
	if v ~= v then return 0 end
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

local function randSimple(seed)
	return (math.sin(seed * 43758.5453) % 1 + 1) % 1 -- [0, 1)
end

local function tempGripFactor(compound, temp)
	local data = COMPOUND_DATA[compound] or COMPOUND_DATA.Street
	local t_min, t_max = data[4], data[5]
	if temp < t_min then
		return 0.5 + 0.5 * math.clamp(temp / math.max(t_min, 1), 0, 1)
	elseif temp <= t_max then
		return 1.0
	elseif temp <= 150 then
		return 1.0 - (temp - t_max) / math.max(150 - t_max, 1) * 0.6
	else
		return 0.3
	end
end

local function pressureGripFactor(pressure, nominal)
	if nominal <= 0 then return 1.0 end
	local ratio = pressure / nominal
	if ratio < 0.5 then return 0.5
	elseif ratio < 0.7 then return 0.7
	elseif ratio <= 1.5 then return 1.0
	else return 0.7 end
end

-- Pacejka magic formula: D * sin(C * atan(B*x - E*(B*x - atan(B*x))))
local function magicFormula(slip, b, c, d, e)
	local s = math.clamp(slip, -4, 4)
	local sb = math.max(b, 0)
	local sc = math.max(c, 0)
	local sd = math.max(d, 0)
	local se = math.clamp(e, -1, 1)
	local bx = sb * s
	local val = sd * math.sin(sc * math.atan(bx - se * (bx - math.atan(bx))))
	return safeFinite(val)
end

-- ============================================================
-- Pacejka force computation
-- ============================================================

function Tires.calculatePacejkaForces(params, slip_ratio, slip_angle, wheel_load, available_grip)
	if wheel_load <= 0 or available_grip <= 0 then
		return { longitudinal = 0, lateral = 0, longitudinal_peak = 0, lateral_peak = 0, combined_magnitude = 0, combined_limit = 0 }
	end

	-- Load sensitivity
	local normalized_load = wheel_load / REFERENCE_TIRE_LOAD
	local load_factor = math.clamp(1.0 - 0.18 * (normalized_load - 1.0), 0.65, 1.15)

	-- Peak forces
	local long_peak = wheel_load * available_grip * load_factor * params.longitudinal_d
	local lat_peak = wheel_load * available_grip * load_factor * params.lateral_d

	-- Raw forces via magic formula (D=1.0 since peak is pre-multiplied)
	local long_force = long_peak * magicFormula(slip_ratio, params.longitudinal_b, params.longitudinal_c, 1.0, params.longitudinal_e)
	local lat_force = -(lat_peak * magicFormula(slip_angle, params.lateral_b, params.lateral_c, 1.0, params.lateral_e))

	-- Combined slip (friction ellipse)
	local norm_l = if long_peak > 0 then math.abs(long_force) / long_peak else 0
	local norm_s = if lat_peak > 0 then math.abs(lat_force) / lat_peak else 0
	local demand = math.sqrt(norm_l * norm_l + norm_s * norm_s)
	if demand > 1.0 then
		long_force = long_force / demand
		lat_force = lat_force / demand
	end

	local combined_mag = math.min(math.sqrt(long_force^2 + lat_force^2), math.max(long_peak, lat_peak))
	local combined_limit = math.sqrt(long_peak^2 + lat_peak^2)

	return {
		longitudinal = safeFinite(long_force),
		lateral = safeFinite(lat_force),
		longitudinal_peak = safeFinite(long_peak),
		lateral_peak = safeFinite(lat_peak),
		combined_magnitude = safeFinite(combined_mag),
		combined_limit = safeFinite(combined_limit),
	}
end

-- Convenience wrapper with default Pacejka parameters.
function Tires.calculateForces(slip_ratio, slip_angle, wheel_load, available_grip)
	return Tires.calculatePacejkaForces(Tires.newPacejkaParams(), slip_ratio, slip_angle, wheel_load, available_grip)
end

-- ============================================================
-- Tire update (thermal, wear, damage)
-- ============================================================

function Tires.update(tire, thermal, slip_ratio, slip_angle, load, surface_roughness, ambient_temp, dt)
	if tire.is_flat then return end

	-- Thermal model
	local slip_energy = math.abs(slip_ratio) * load * tire.grip_factor
	local heat_in = thermal.heat_generation_rate * slip_energy * dt / thermal.thermal_mass
	local heat_out = thermal.heat_dissipation_rate * (tire.temperature - ambient_temp) * dt / thermal.thermal_mass
	tire.temperature = math.max(tire.temperature + heat_in - heat_out, ambient_temp - 10.0)

	-- Pressure (ideal gas at constant volume)
	local NOMINAL_TEMP_K = 293.15
	tire.pressure = tire.nominal_pressure * math.max((tire.temperature + 273.15) / NOMINAL_TEMP_K, 0)

	-- Wear
	local slip_wear = math.min(math.abs(slip_ratio) * 0.3 + math.abs(slip_angle) * 0.3, 1.0)
	local load_factor = math.clamp(load / 5000, 0.5, 2.0)
	local data = COMPOUND_DATA[tire.compound] or COMPOUND_DATA.Street
	local wear_rate = data[3]
	tire.wear = math.min(tire.wear + slip_wear * load_factor * surface_roughness * wear_rate * 0.0001 * dt, 1.0)

	-- Center vs shoulder wear
	local cornering_fraction = math.abs(slip_angle) / (math.abs(slip_ratio) + math.abs(slip_angle) + 1e-6)
	tire.center_wear = tire.center_wear + (1 - cornering_fraction) * slip_wear * load_factor * surface_roughness * wear_rate * 0.00005 * dt
	tire.shoulder_wear = tire.shoulder_wear + cornering_fraction * slip_wear * load_factor * surface_roughness * wear_rate * 0.00005 * dt

	-- Flat spot from lockup
	if slip_ratio < -0.5 and load > 1000 then
		tire.center_wear = tire.center_wear + 0.001 * dt
	end

	-- Damage events
	if tire.wear > 0.85 then tire.cord_exposed = true end

	if tire.wear > 0.7 and surface_roughness > 0.5 then
		local puncture_chance = (tire.wear - 0.7) * surface_roughness * 0.5 * dt
		if randSimple(tire.wear * 1000 + tire.temperature) < puncture_chance then
			tire.has_puncture = true
			tire.wear = math.min(tire.wear + 0.1, 1.0)
		end
	end

	tire.blowout_risk = math.max(
		(tire.wear - 0.6) * 1.5
		+ math.abs(tire.pressure / math.max(tire.nominal_pressure, 1) - 1) * 0.5
		+ (if tire.temperature > 130 then 0.3 else 0),
		0
	)

	if tire.blowout_risk > 1.0 then
		tire.is_flat = true
		tire.pressure = 0
	end

	if tire.wear > 0.95 then
		tire.is_flat = true
	end

	if tire.temperature > 120 then
		tire.has_blister = true
	end

	-- Grip factor (combined)
	local tg = tempGripFactor(tire.compound, tire.temperature)
	local pg = pressureGripFactor(tire.pressure, tire.nominal_pressure)
	local data2 = COMPOUND_DATA[tire.compound] or COMPOUND_DATA.Street
	tire.grip_factor = math.clamp(
		data2[1] * tg * pg * (1 - tire.wear * 0.7) * (if tire.has_puncture then 0.5 else 1.0),
		0, 1
	)

	if tire.has_blister then tire.grip_factor = tire.grip_factor * 0.9 end
	if tire.is_flat then tire.grip_factor = 0.2 end
end

-- ============================================================
-- Telemetry
-- ============================================================

function Tires.telemetry(tire)
	return {
		temperature = tire.temperature,
		pressure = tire.pressure,
		wear = tire.wear,
		grip_factor = tire.grip_factor,
		is_flat = tire.is_flat,
		center_wear = tire.center_wear,
		shoulder_wear = tire.shoulder_wear,
	}
end

return Tires
