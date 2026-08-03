--[[
	Capability Manifest — auto-generates what's configurable per vehicle.
	Builds from the vehicle asset loader output. If a subsystem exists in
	the asset, it becomes configurable. Ranges are derived from authored
	values with sensible defaults. Authors can override via ConfigManifest
	Folder in the vehicle asset.

	Pure Luau, no Roblox API dependencies.
--]]

local Manifest = {}

-- ============================================================
-- Range derivation helpers
-- ============================================================

--- Derive a [min, max, step] range from a current value.
--- Conservative: ±50% for most params, ±30% for safety-critical.
local function autoRange(value, opts)
	opts = opts or {}
	local scale = opts.scale or 0.5
	local floor = opts.floor or 0
	local ceil = opts.ceil or math.huge
	local stepFraction = opts.step_fraction or 0.05
	if type(value) ~= "number" or value ~= value then return nil end
	local lo = math.max(value * (1 - scale), floor)
	local hi = math.min(value * (1 + scale * 2), ceil)
	local step = math.max(value * stepFraction, opts.step_min or 1)
	-- Snap step to a clean number
	if step > 100 then step = math.floor(step / 100) * 100
	elseif step > 10 then step = math.floor(step / 10) * 10
	elseif step > 1 then step = math.floor(step)
	else step = math.floor(step * 10) / 10
	end
	if step < (opts.step_min or 1) then step = opts.step_min or 1 end
	return { min = lo, max = hi, step = step, default = value }
end

--- For small decimals (0-1 range).
local function decimalRange(value, opts)
	opts = opts or {}
	local lo = math.max((value or 0) * 0.2, opts.floor or -1)
	local hi = math.min((value or 0) * 3 + 0.1, opts.ceil or 10)
	local step = 0.01
	return { min = lo, max = hi, step = step, default = value or 0 }
end

local WHEEL_IDS = { "FL", "FR", "RL", "RR" }

-- ============================================================
-- Category builders
-- ============================================================

local function buildEngine(engineConfig, simConfig)
	if not engineConfig then return nil end
	local params = {}
	params.idle_rpm = autoRange(engineConfig.idle_rpm, { floor = 400, ceil = 1500, step_min = 25 })
	params.redline_rpm = autoRange(engineConfig.redline_rpm, { floor = 4000, ceil = 12000, step_min = 50 })
	params.rev_limiter_rpm = autoRange(engineConfig.rev_limiter_rpm, { floor = 4200, ceil = 13000, step_min = 50 })
	params.throttle_response = decimalRange(engineConfig.throttle_response, { floor = 0.1, ceil = 3.0 })
	params.engine_braking = decimalRange(engineConfig.engine_braking, { floor = 0.05, ceil = 1.0 })

	-- Torque curve exposed as presets, not raw editing
	local peakTorque = 0
	if engineConfig.torque_curve then
		for _, pt in engineConfig.torque_curve do
			if pt[2] > peakTorque then peakTorque = pt[2] end
		end
	end
	params.torque_preset = {
		options = { "stock", "sport", "aggressive" },
		default = "stock",
		peak_torque_stock = peakTorque,
	}

	return {
		editable = true,
		params = params,
	}
end

local function buildTurbo(simConfig)
	local turboData = simConfig and simConfig.turbo
	if not turboData then return nil end
	-- If turbo type is explicitly "None" and there's no boost params, skip
	local hasContent = turboData.type and turboData.type ~= "None"
	if not hasContent and not turboData.max_boost_psi then return nil end

	local params = {}
	params.enabled = {
		options = { true, false },
		default = true,
	}
	params.max_boost_psi = autoRange(turboData.max_boost_psi or 10, {
		floor = 2, ceil = 40, step_min = 0.5, scale = 0.3,
	})
	params.spool_time = autoRange(turboData.spool_time or 1.0, {
		floor = 0.1, ceil = 5.0, step_min = 0.05, scale = 0.4,
	})

	return {
		editable = true,
		removable = true,
		params = params,
		current_type = turboData.type or "None",
	}
end

local function buildCooling(simConfig)
	-- Cooling params live in the thermal system, not directly in simulation_config
	-- We detect cooling capability from the engine thermal config
	local params = {
		radiator_capacity = { min = 0, max = 10.0, step = 0.5, default = 4.0 },
		coolant_capacity = { min = 0.5, max = 8.0, step = 0.5, default = 3.0 },
		fan_temp = { min = 75, max = 115, step = 5, default = 95 },
	}
	return {
		editable = true,
		removable = true,
		params = params,
	}
end

local function buildSuspension(suspensionConfig)
	if not suspensionConfig or not suspensionConfig.wheels then return nil end
	local perWheel = {}
	for i = 1, math.min(#suspensionConfig.wheels, 4) do
		local w = suspensionConfig.wheels[i]
		perWheel[WHEEL_IDS[i]] = {
			spring_rate = autoRange(w.spring_rate, { floor = 5000, ceil = 100000, step_min = 500 }),
			damping = autoRange(w.damping, { floor = 500, ceil = 15000, step_min = 100 }),
			rebound_damping = autoRange(w.rebound_damping, { floor = 500, ceil = 12000, step_min = 100 }),
			travel = autoRange(w.travel, { floor = 0.03, ceil = 0.50, step_min = 0.01 }),
			tire_radius = autoRange(w.tire_radius, { floor = 0.15, ceil = 0.70, step_min = 0.01 }),
		}
	end

	return {
		editable = true,
		per_wheel = true,
		per_wheel_params = perWheel,
		shared_params = {
			anti_roll_bar_stiffness = autoRange(
				suspensionConfig.anti_roll_bar_stiffness or 2000,
				{ floor = 0, ceil = 20000, step_min = 100 }
			),
		},
	}
end

local function buildBrakes(simConfig)
	local brakeData = simConfig and simConfig.brakes
	if not brakeData then return nil end
	-- Brake config is a single table (not per-wheel in simulation_config)
	-- The per-wheel brake_torque_max comes from the JSON asset
	local params = {
		brake_torque_max = autoRange(brakeData.brake_torque_max or 3000, {
			floor = 500, ceil = 10000, step_min = 100,
		}),
	}
	return {
		editable = true,
		params = params,
	}
end

local function buildTransmission(drivetrainConfig)
	if not drivetrainConfig or not drivetrainConfig.transmission then return nil end
	local trans = drivetrainConfig.transmission
	local params = {
		final_drive = autoRange(trans.final_drive, { floor = 1.5, ceil = 8.0, step_min = 0.1, scale = 0.3 }),
	}
	-- Gear ratios as presets to avoid complex interdependency
	params.gear_preset = {
		options = { "short", "stock", "tall" },
		default = "stock",
		stock_ratios = trans.gear_ratios,
	}
	return {
		editable = true,
		params = params,
	}
end

local function buildAerodynamics(simConfig)
	local aero = simConfig and simConfig.aero
	if not aero then return nil end
	return {
		editable = true,
		params = {
			cd = decimalRange(aero.cd, { floor = 0.10, ceil = 0.80 }),
			cl = decimalRange(aero.cl, { floor = -0.80, ceil = 0.50 }),
		},
	}
end

-- ============================================================
-- Public API
-- ============================================================

--- Build a capability manifest from the vehicle asset loader output.
--- @param definition table  — output of VehicleLoader.load(model)
--- @return table  — capability manifest
function Manifest.build(definition)
	local manifest = {
		version = 1,
		categories = {},
	}

	-- Engine
	local engine = buildEngine(definition.engineConfig, definition.simulationConfig)
	if engine then manifest.categories.engine = engine end

	-- Turbo (only if vehicle has turbo hardware)
	local turbo = buildTurbo(definition.simulationConfig)
	if turbo then manifest.categories.turbo = turbo end

	-- Cooling
	local cooling = buildCooling(definition.simulationConfig)
	if cooling then manifest.categories.cooling = cooling end

	-- Suspension
	local suspension = buildSuspension(definition.suspensionConfig)
	if suspension then manifest.categories.suspension = suspension end

	-- Brakes
	local brakes = buildBrakes(definition.simulationConfig)
	if brakes then manifest.categories.brakes = brakes end

	-- Transmission
	local transmission = buildTransmission(definition.drivetrainConfig)
	if transmission then manifest.categories.transmission = transmission end

	-- Aerodynamics
	local aero = buildAerodynamics(definition.simulationConfig)
	if aero then manifest.categories.aerodynamics = aero end

	return manifest
end

--- Validate a single parameter change against the manifest.
--- @param manifest table  — capability manifest
--- @param category string  — e.g. "engine", "suspension"
--- @param param string  — e.g. "redline_rpm", "spring_rate"
--- @param value any  — proposed new value
--- @param wheelId string|nil  — "FL"/"FR"/"RL"/"RR" for per-wheel params
--- @return boolean, string|nil  — ok, error_message
function Manifest.validate(manifest, category, param, value, wheelId)
	local cat = manifest.categories[category]
	if not cat then
		return false, "category '" .. category .. "' not available on this vehicle"
	end
	if not cat.editable then
		return false, "category '" .. category .. "' is not editable"
	end

	-- Resolve the param definition
	local paramDef = nil
	if cat.per_wheel then
		if not wheelId then
			return false, "per-wheel parameter requires wheelId (FL/FR/RL/RR)"
		end
		if not cat.per_wheel_params or not cat.per_wheel_params[wheelId] then
			return false, "wheel '" .. wheelId .. "' not found"
		end
		paramDef = cat.per_wheel_params[wheelId][param]
		-- Fall back to shared params
		if not paramDef and cat.shared_params then
			paramDef = cat.shared_params[param]
		end
	elseif cat.params then
		paramDef = cat.params[param]
	end

	if not paramDef then
		return false, "parameter '" .. param .. "' not found in '" .. category .. "'"
	end

	-- Options-based param (preset, compound, etc.)
	if paramDef.options then
		local found = false
		for _, opt in paramDef.options do
			if opt == value then found = true break end
		end
		if not found then
			return false, "value '" .. tostring(value) .. "' not in options for '" .. param .. "'"
		end
		return true, nil
	end

	-- Numeric range param
	if paramDef.min ~= nil and paramDef.max ~= nil then
		if type(value) ~= "number" or value ~= value then
			return false, "parameter '" .. param .. "' requires a number"
		end
		if value < paramDef.min then
			return false, string.format("value %.2f below minimum %.2f for '%s'", value, paramDef.min, param)
		end
		if value > paramDef.max then
			return false, string.format("value %.2f above maximum %.2f for '%s'", value, paramDef.max, param)
		end
		return true, nil
	end

	-- Boolean param
	if type(paramDef.default) == "boolean" then
		if type(value) ~= "boolean" then
			return false, "parameter '" .. param .. "' requires a boolean"
		end
		return true, nil
	end

	return false, "cannot validate parameter '" .. param .. "'"
end

--- Snapshot current values from the world for a given manifest.
--- @param world table  — the physics World instance
--- @param manifest table  — capability manifest
--- @return table  — { [category] = { [param] = currentValue, ... } }
function Manifest.snapshot(world, manifest)
	local result = {}
	local function set(cat, param, value)
		if not result[cat] then result[cat] = {} end
		result[cat][param] = value
	end

	local cat = manifest.categories

	-- Engine
	if cat.engine and world.drivetrain and world.drivetrain.engine then
		local e = world.drivetrain.engine
		set("engine", "idle_rpm", e.idle_rpm)
		set("engine", "redline_rpm", e.redline_rpm)
		set("engine", "rev_limiter_rpm", e.rev_limiter_rpm)
		set("engine", "throttle_response", e.throttle_response)
		set("engine", "engine_braking", e.engine_braking)
	end

	-- Turbo
	if cat.turbo and world.turbo_config then
		set("turbo", "max_boost_psi", world.turbo_config.max_boost_psi or 0)
		set("turbo", "spool_time", world.turbo_config.spool_time or 1)
	end

	-- Cooling
	if cat.cooling and world.engine_thermal and world.engine_thermal.cooling then
		set("cooling", "radiator_capacity", world.engine_thermal.cooling.radiator_capacity)
		set("cooling", "coolant_capacity", world.engine_thermal.cooling.coolant_capacity or 0)
		set("cooling", "fan_temp", world.engine_thermal.cooling.fan_activation_temp)
	end

	-- Suspension
	if cat.suspension and world.suspension_config then
		for i = 1, math.min(#world.suspension_config.wheels, 4) do
			local w = world.suspension_config.wheels[i]
			local id = WHEEL_IDS[i]
			if cat.suspension.per_wheel_params and cat.suspension.per_wheel_params[id] then
				set("suspension_" .. id, "spring_rate", w.spring_rate)
				set("suspension_" .. id, "damping", w.damping)
				set("suspension_" .. id, "rebound_damping", w.rebound_damping)
				set("suspension_" .. id, "travel", w.travel)
				set("suspension_" .. id, "tire_radius", w.tire_radius)
			end
		end
		if cat.suspension.shared_params then
			set("suspension", "anti_roll_bar_stiffness", world.suspension_config.anti_roll_bar_stiffness)
		end
	end

	-- Brakes
	if cat.brakes and world.brake_config then
		set("brakes", "brake_torque_max", world.brake_config.brake_torque_max or 3000)
	end

	-- Transmission
	if cat.transmission and world.drivetrain and world.drivetrain.transmission then
		set("transmission", "final_drive", world.drivetrain.transmission.final_drive)
	end

	-- Aerodynamics
	if cat.aerodynamics then
		set("aerodynamics", "cd", world.aeroCd or 0)
		set("aerodynamics", "cl", world.aeroCl or 0)
	end

	return result
end

return Manifest
