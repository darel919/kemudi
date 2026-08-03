--[[
	Vehicle Configurator — validates and applies real-time config changes.
	Consumes a capability manifest, validates user requests, mutates the
	running World's config tables, and handles structural resets.

	Pure Luau, no Roblox API dependencies.
--]]

local Manifest = if script then require(script.Parent.manifest) else require("./manifest")
local Presets = if script then require(script.Parent.param_presets) else require("./param_presets")

local Configurator = {}
Configurator.__index = Configurator

local WHEEL_IDS = { "FL", "FR", "RL", "RR" }

-- ============================================================
-- Constructor
-- ============================================================

function Configurator.new(manifest, world)
	local self = setmetatable({}, Configurator)
	self.manifest = manifest
	self.world = world
	self.defaults = Presets.captureDefaults(world, manifest)
	self.change_log = {}  -- { { category, param, wheelId, old, new, tick }, ... }
	self.change_count = 0
	return self
end

-- ============================================================
-- Validate
-- ============================================================

--- Validate a proposed config change.
--- @param category string
--- @param param string
--- @param value any
--- @param wheelId string|nil
--- @return boolean, string|nil  — ok, error
function Configurator:validate(category, param, value, wheelId)
	return Manifest.validate(self.manifest, category, param, value, wheelId)
end

-- ============================================================
-- Apply
-- ============================================================

--- Apply a validated config change to the running World.
--- @param category string
--- @param param string
--- @param value any
--- @param wheelId string|nil
--- @return boolean, string|nil  — ok, error
function Configurator:apply(category, param, value, wheelId)
	local ok, err = self:validate(category, param, value, wheelId)
	if not ok then return false, err end

	local world = self.world
	local old_value = nil

	-- ==================== ENGINE ====================
	if category == "engine" then
		local e = world.drivetrain and world.drivetrain.engine
		if not e then return false, "engine not initialized" end

		if param == "torque_preset" then
			-- Preset change: replace the torque curve
			local preset = Presets.getTorqueCurve(value, self.defaults.engine and self.defaults.engine.torque_preset_peak)
			-- We need the original curve; grab from the default snapshot or keep it
			if world.drivetrain.tcm and world.drivetrain.tcm.torque_curve then
				local baseCurve = world.drivetrain.tcm.torque_curve
				local newCurve = Presets.getTorqueCurve(value, baseCurve)
				if newCurve then
					old_value = "stock"
					e.torque_curve = newCurve
					world.drivetrain.tcm.torque_curve = newCurve
					-- Recompute peak torque for TCM
					local peak = 0
					for _, pt in newCurve do if pt[2] > peak then peak = pt[2] end end
					world.drivetrain.tcm.peak_torque = peak
				end
			end
		else
			-- Scalar engine param
			old_value = e[param]
			if old_value == nil then return false, "unknown engine param '" .. param .. "'" end
			e[param] = value

			-- Derived: if redline changed, ensure rev_limiter is above it
			if param == "redline_rpm" and e.rev_limiter_rpm < value then
				e.rev_limiter_rpm = value + 200
			end
		end

	-- ==================== TURBO ====================
	elseif category == "turbo" then
		if not world.turbo_config then return false, "vehicle has no turbo" end

		if param == "enabled" then
			-- Toggle turbo on/off
			old_value = world.turbo_config.type ~= "None"
			if value == true then
				-- Re-enable: use the manifest's current_type if available
				local cat = self.manifest.categories.turbo
				world.turbo_config.type = (cat and cat.current_type) or "Turbo"
			else
				world.turbo_config.type = "None"
				-- Reset turbo state
				if world.turbo_state then
					world.turbo_state.boost_pressure = 0
					world.turbo_state.spool_position = 0
					world.turbo_state.is_boosting = false
				end
			end
		else
			old_value = world.turbo_config[param]
			if old_value == nil then return false, "unknown turbo param '" .. param .. "'" end
			world.turbo_config[param] = value
		end

	-- ==================== COOLING ====================
	elseif category == "cooling" then
		local cooling = world.engine_thermal and world.engine_thermal.cooling
		if not cooling then return false, "cooling system not initialized" end

		if param == "radiator_capacity" then
			old_value = cooling.radiator_capacity
			cooling.radiator_capacity = value
			-- Zero radiator → damage_factor drops proportionally
			-- The coolingRate() function handles radiator_capacity=0 natively
		elseif param == "coolant_capacity" then
			old_value = world.engine_thermal.thermal and world.engine_thermal.thermal.coolant_capacity
			if world.engine_thermal.thermal then
				world.engine_thermal.thermal.coolant_capacity = value
			end
		elseif param == "fan_temp" then
			old_value = cooling.fan_activation_temp
			cooling.fan_activation_temp = value
			-- Maintain hysteresis: fan_off = activation - 5
			cooling.fan_deactivation_temp = value - 5
		else
			return false, "unknown cooling param '" .. param .. "'"
		end

	-- ==================== SUSPENSION ====================
	elseif category == "suspension" then
		if not world.suspension_config then return false, "suspension not initialized" end

		-- Shared params
		if param == "anti_roll_bar_stiffness" then
			old_value = world.suspension_config.anti_roll_bar_stiffness
			world.suspension_config.anti_roll_bar_stiffness = value
			return true, nil
		end

		-- Per-wheel params
		if not wheelId then return false, "suspension params require wheelId" end
		local wheelIndex = nil
		for i, id in WHEEL_IDS do
			if id == wheelId then wheelIndex = i break end
		end
		if not wheelIndex then return false, "invalid wheelId '" .. wheelId .. "'" end

		local wheel = world.suspension_config.wheels[wheelIndex]
		if not wheel then return false, "wheel " .. wheelIndex .. " not found in config" end
		old_value = wheel[param]
		if old_value == nil then return false, "unknown suspension param '" .. param .. "'" end
		wheel[param] = value

		-- Sync tire_radius to wheel_dynamics config
		if param == "tire_radius" and world.wheel_dynamics_configs then
			local wdConfig = world.wheel_dynamics_configs[wheelIndex]
			if wdConfig then
				wdConfig.tire_radius = value
				-- Recompute inertia
				local wdState = world.wheel_dynamics_states and world.wheel_dynamics_states[wheelIndex]
				if wdState then
					wdState.tire_radius = value
					local mass = assert(wdConfig.wheel_mass, "wheel mass is missing from vehicle asset")
					wdState.inertia = 0.5 * mass * value * value
				end
			end
			-- Sync drivetrain wheel_radius
			if world.drivetrain and wheelIndex == 1 then
				world.drivetrain.wheel_radius = value
			end
		end

	-- ==================== BRAKES ====================
	elseif category == "brakes" then
		if not world.brake_config then return false, "brakes not initialized" end
		old_value = world.brake_config[param]
		if old_value == nil then return false, "unknown brake param '" .. param .. "'" end
		world.brake_config[param] = value

	-- ==================== TRANSMISSION ====================
	elseif category == "transmission" then
		if not world.drivetrain or not world.drivetrain.transmission then
			return false, "transmission not initialized"
		end
		local trans = world.drivetrain.transmission

		if param == "gear_preset" then
			-- Gear ratio preset
			local cat = self.manifest.categories.transmission
			local baseRatios = (cat and cat.params and cat.params.gear_preset
				and cat.params.gear_preset.stock_ratios) or trans.gear_ratios
			local newRatios, newFinalDrive = Presets.getGearPreset(value, baseRatios, trans.final_drive)
			if newRatios then
				old_value = "stock"
				trans.gear_ratios = newRatios
				if newFinalDrive then trans.final_drive = newFinalDrive end
				-- Sync TCM gear ratios
				if world.drivetrain.tcm then
					world.drivetrain.tcm.gear_ratios = newRatios
					world.drivetrain.tcm.final_drive = trans.final_drive
				end
			end
		else
			old_value = trans[param]
			if old_value == nil then return false, "unknown transmission param '" .. param .. "'" end
			trans[param] = value
			-- Sync TCM
			if world.drivetrain.tcm then
				world.drivetrain.tcm[param] = value
			end
		end

	-- ==================== AERODYNAMICS ====================
	elseif category == "aerodynamics" then
		if param == "cd" then
			old_value = world.aeroCd
			world.aeroCd = value
			world.dragCoefficient = value
		elseif param == "cl" then
			old_value = world.aeroCl
			world.aeroCl = value
		else
			return false, "unknown aero param '" .. param .. "'"
		end

	else
		return false, "unknown category '" .. category .. "'"
	end

	-- Log the change
	self.change_count += 1
	table.insert(self.change_log, {
		category = category,
		param = param,
		wheelId = wheelId,
		old = old_value,
		new = value,
		tick = self.change_count,
	})

	return true, nil
end

-- ============================================================
-- Snapshot
-- ============================================================

--- Get a snapshot of current config values.
--- @return table
function Configurator:snapshot()
	return Manifest.snapshot(self.world, self.manifest)
end

-- ============================================================
-- Reset to defaults
-- ============================================================

--- Reset all params to their original stock values.
--- @return boolean, string|nil
function Configurator:resetAll()
	if not self.defaults then return false, "no defaults captured" end
	for category, params in self.defaults do
		-- Strip the "suspension_FL" etc. prefix for per-wheel
		local baseCategory = category
		local wheelId = nil
		if category:sub(1, 12) == "suspension_" then
			baseCategory = "suspension"
			wheelId = category:sub(13) -- "FL", "FR", etc.
		end
		for param, value in params do
			-- Skip preset metadata
			if param ~= "torque_preset_peak" then
				self:apply(baseCategory, param, value, wheelId)
			end
		end
	end
	return true, nil
end

--- Reset a single category to defaults.
--- @param category string
--- @return boolean, string|nil
function Configurator:resetCategory(category)
	if not self.defaults then return false, "no defaults captured" end
	-- Check direct match or suspension_ prefix
	for key, params in self.defaults do
		local baseCategory = key
		local wheelId = nil
		if key:sub(1, 12) == "suspension_" then
			baseCategory = "suspension"
			wheelId = key:sub(13)
		end
		if baseCategory == category then
			for param, value in params do
				if param ~= "torque_preset_peak" then
					self:apply(baseCategory, param, value, wheelId)
				end
			end
		end
	end
	return true, nil
end

-- ============================================================
-- Change log (for UI diff display / network sync)
-- ============================================================

--- Get changes since a given tick.
--- @param sinceTick number
--- @return table  — array of changes
function Configurator:getChangesSince(sinceTick)
	local changes = {}
	for _, change in self.change_log do
		if change.tick > sinceTick then
			table.insert(changes, change)
		end
	end
	return changes
end

return Configurator
