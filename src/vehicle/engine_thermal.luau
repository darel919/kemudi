--[[
	Engine thermal, lubrication, damage, and stress simulation.
	Ported from crates/kemudi-engine/src/engine/*.
	Pure Luau, no Roblox API dependencies.
--]]

local EngineThermal = {}

-- ============================================================
-- Defaults matching Rust EngineThermal::default()
-- ============================================================

function EngineThermal.newThermal(overrides)
	local t = {
		coolant_temp = 20.0,
		oil_temp = 20.0,
		transmission_temp = 20.0,
		ambient_temp = 25.0,
		coolant_capacity = 1.5, -- Effective thermal mass (engine block + coolant in block), not total system capacity
		oil_capacity = 1.0, -- Effective oil thermal mass in engine
		coolant_specific_heat = 4186.0,
		oil_specific_heat = 2000.0,
		combustion_heat_rate = 50000.0, -- kW at full load (was 5000 - too low by 10x)
		friction_heat_rate = 8000.0, -- kW at redline (was 800 - too low by 10x)
		exhaust_heat_fraction = 0.35,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

function EngineThermal.newCoolingSystem(overrides)
	local t = {
		radiator_capacity = 3000.0, -- Reduced from 8000 for more realistic cooling
		fan_activation_temp = 95.0,
		fan_deactivation_temp = 90.0, -- hysteresis: fan turns off below this
		fan_additional_cooling = 3000.0,
		thermostat_open_temp = 82.0,
		thermostat_range = 13.0,
		coolant_flow_rate = 1.0,
		damage_factor = 1.0,
		-- Runtime state (not authored)
		fan_active = false,
		fan_speed = 0.0, -- 0-1 proportional
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

function EngineThermal.newLubricationSystem(overrides)
	local t = {
		oil_pressure = 300.0,
		nominal_pressure = 300.0,
		pump_pressure = 400.0,
		pickup_position = 0.0,
		oil_level = 1.0,
		is_leaking = false,
		leak_rate = 0.0,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

function EngineThermal.newDamage(overrides)
	local t = {
		wear_level = 0.0,
		overheating_cycles = 0,
		overrev_cycles = 0,
		knock_events = 0,
		has_coolant_leak = false,
		has_oil_leak = false,
		head_gasket_failed = false,
		bearing_damage = 0.0,
		is_seized = false,
		is_on_fire = false,
		fire_timer = 0.0,
		lugging_events = 0,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

function EngineThermal.newStressAccumulators(overrides)
	local t = {
		thermal_stress = 0.0,
		oil_stress = 0.0,
		overrev_stress = 0.0,
		lugging_stress = 0.0,
		thermal_cooldown_rate = 0.1,
		oil_cooldown_rate = 0.15,
		overrev_cooldown_rate = 0.05,
		lugging_cooldown_rate = 0.1,
		thermal_limit = 1.0,
		oil_limit = 1.0,
		overrev_limit = 1.0,
		lugging_limit = 1.0,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

-- ============================================================
-- Cooling system helpers
-- ============================================================

function EngineThermal.thermostatValve(cooling, coolant_temp)
	if coolant_temp <= cooling.thermostat_open_temp then return 0 end
	if coolant_temp >= cooling.thermostat_open_temp + cooling.thermostat_range then return 1 end
	return (coolant_temp - cooling.thermostat_open_temp) / cooling.thermostat_range
end

function EngineThermal.coolingRate(cooling, coolant_temp, vehicle_speed)
	local valve = math.clamp(EngineThermal.thermostatValve(cooling, coolant_temp), 0, 1)
	local speed = if type(vehicle_speed) == "number" and vehicle_speed == vehicle_speed then math.abs(vehicle_speed) else 0
	local airflow = 0.3 + 0.7 * math.clamp(speed / 30.0, 0, 1)
	local damage = math.max(cooling.damage_factor, 0)
	local rate = math.max(cooling.radiator_capacity, 0) * valve * airflow * damage

	-- Fan with hysteresis: ON at fan_activation_temp, OFF at fan_deactivation_temp
	-- Proportional speed between deactivation and activation temps
	local fan_deact = cooling.fan_deactivation_temp or (cooling.fan_activation_temp - 5)
	if cooling.fan_active then
		-- Currently on: stays on until temp drops below deactivation threshold
		if coolant_temp < fan_deact then
			cooling.fan_active = false
			cooling.fan_speed = 0
		else
			-- Proportional: full speed at activation temp, ramps down toward deactivation
			local range = math.max(cooling.fan_activation_temp - fan_deact, 1)
			cooling.fan_speed = math.clamp((coolant_temp - fan_deact) / range, 0, 1)
		end
	else
		-- Currently off: turns on only above activation threshold
		if coolant_temp >= cooling.fan_activation_temp then
			cooling.fan_active = true
			cooling.fan_speed = 1.0
		else
			cooling.fan_speed = 0
		end
	end

	if cooling.fan_active then
		rate += math.max(cooling.fan_additional_cooling, 0) * damage * cooling.fan_speed
	end

	return rate * math.max(cooling.coolant_flow_rate, 0)
end

-- ============================================================
-- Lubrication update
-- ============================================================

function EngineThermal.updateLubrication(lube, accel_g, dt, rpm)
	local safe_accel = if type(accel_g) == "number" and accel_g == accel_g then accel_g else 0
	lube.pickup_position = math.clamp(safe_accel * 0.3, -1, 1)
	if lube.pickup_position < 0 then lube.pickup_position = 0 end

	if lube.is_leaking then
		lube.oil_level = math.max(lube.oil_level - math.max(lube.leak_rate, 0) * math.max(dt, 0), 0)
	end

	local level_factor = math.clamp(lube.oil_level, 0, 1)
	local pickup_factor = math.max(1 - lube.pickup_position * 0.7, 0)
	local pump_factor = 0
	if type(rpm) == "number" and rpm > 0 then
		pump_factor = math.clamp(rpm / 800.0, 0, 1)
	end
	local target_pressure = math.max(lube.pump_pressure, 0)
		* math.clamp(pump_factor, 0, 1)
		* level_factor
		* pickup_factor
	if pump_factor > 0 then
		lube.oil_pressure = target_pressure
	else
		local safe_dt = math.max(dt, 0)
		local pressure_decay = math.max(1 - 8.0 * safe_dt, 0)
		lube.oil_pressure = math.max(lube.oil_pressure * pressure_decay, 0)
	end
end

-- ============================================================
-- Damage checks
-- ============================================================

function EngineThermal.checkOverheat(damage, coolant_temp)
	if coolant_temp >= 115 then -- Changed from > 115 to >= 115
		damage.overheating_cycles += 1
	end
	if coolant_temp > 130 and not damage.head_gasket_failed then -- Raised from 125
		if damage.overheating_cycles > 200 then -- Raised from 100
			damage.head_gasket_failed = true
			damage.has_coolant_leak = true
		end
	end
	if coolant_temp > 150 then -- Raised from 140
		damage.wear_level = math.min(damage.wear_level + 0.002, 1) -- Reduced from 0.005
		if damage.wear_level > 0.8 and not damage.is_on_fire then
			damage.fire_timer += 0.01
			if damage.fire_timer > 1 then
				damage.is_on_fire = true
			end
		end
	end
end

function EngineThermal.checkOverrev(damage, rpm, redline)
	if rpm > redline * 1.1 then
		damage.overrev_cycles += 1
		damage.wear_level = math.min(damage.wear_level + 0.00005, 1) -- Reduced from 0.0001
	end
end

function EngineThermal.checkOilStarvation(damage, pressure)
	if pressure < 50 then
		damage.bearing_damage = math.min(damage.bearing_damage + 0.0005, 1) -- Reduced from 0.001
	end
	if pressure < 10 then
		damage.bearing_damage = math.min(damage.bearing_damage + 0.005, 1) -- Reduced from 0.01
	end
	if damage.bearing_damage > 0.8 then
		damage.is_seized = true
	end
end

function EngineThermal.checkLugging(damage, rpm, throttle, gear, time)
	if gear >= 3 and rpm < 1500 and throttle > 0.5 then
		damage.lugging_events += 1
		damage.wear_level = math.min(damage.wear_level + 0.0001, 1) -- Reduced from 0.0002
	end
end

function EngineThermal.updateDamage(damage, dt)
	if damage.is_on_fire then
		damage.fire_timer += dt
		damage.wear_level = math.min(damage.wear_level + 0.05 * dt, 1)
	end
end

-- ============================================================
-- Stress accumulators
-- ============================================================

function EngineThermal.accumulateThermal(stress, coolant_temp, redline_temp, dt)
	if coolant_temp > redline_temp then
		local severity = (coolant_temp - redline_temp) / 30.0
		stress.thermal_stress = math.min(stress.thermal_stress + severity * dt, stress.thermal_limit * 2)
	else
		stress.thermal_stress = math.max(stress.thermal_stress - stress.thermal_cooldown_rate * dt, 0)
	end
end

function EngineThermal.accumulateOil(stress, oil_pressure, nominal_pressure, dt)
	if oil_pressure < nominal_pressure * 0.3 then
		local severity = math.min(1 - oil_pressure / (nominal_pressure * 0.3), 2)
		stress.oil_stress = math.min(stress.oil_stress + severity * dt, stress.oil_limit * 2)
	else
		stress.oil_stress = math.max(stress.oil_stress - stress.oil_cooldown_rate * dt, 0)
	end
end

function EngineThermal.accumulateOverrev(stress, rpm, redline, dt)
	if rpm > redline then
		local excess = (rpm - redline) / redline
		stress.overrev_stress = math.min(stress.overrev_stress + excess * dt, stress.overrev_limit * 2)
	else
		stress.overrev_stress = math.max(stress.overrev_stress - stress.overrev_cooldown_rate * dt, 0)
	end
end

function EngineThermal.accumulateLugging(stress, rpm, throttle, gear, dt)
	if gear >= 3 and rpm < 1500 and throttle > 0.5 then
		local severity = throttle * (1 - rpm / 1500)
		stress.lugging_stress = math.min(stress.lugging_stress + severity * dt, stress.lugging_limit * 2)
	else
		stress.lugging_stress = math.max(stress.lugging_stress - stress.lugging_cooldown_rate * dt, 0)
	end
end

function EngineStressBlowupStage(stress)
	local max_stress = math.max(stress.thermal_stress, stress.oil_stress, stress.overrev_stress, stress.lugging_stress)
	if max_stress > 1.8 then return 6 -- CompleteFailure
	elseif max_stress > 1.5 then return 5 -- SeizureOrFire
	elseif max_stress > 1.2 then return 4 -- SeverePowerLoss
	elseif max_stress > 0.9 then return 3 -- RoughRunning
	elseif max_stress > 0.5 then return 2 -- TorqueDerate
	elseif max_stress > 0.2 then return 1 -- WarningLamp
	else return 0 end -- Ok
end

function EngineStressDerateFactor(stress)
	local max_stress = math.max(stress.thermal_stress, stress.oil_stress, stress.overrev_stress, stress.lugging_stress)
	return math.clamp(1 - max_stress * 0.4, 0, 1)
end

-- ============================================================
-- Combined engine update (matches Rust update_engine_full)
-- ============================================================

function EngineThermal.updateFull(thermal, cooling, lube, damage, stress, rpm, redline_rpm, throttle, vehicle_speed, accel_g, gear, dt)
	local safe_redline = if type(redline_rpm) == "number" and redline_rpm > 0 then redline_rpm else 7000
	local safe_throttle = math.clamp(throttle or 0, 0, 1)
	local normalized_rpm = math.clamp((rpm or 0) / safe_redline, 0, 1.5)
	local load = safe_throttle * normalized_rpm

	-- Heat generation
	local combustion_heat = thermal.combustion_heat_rate * load
	local friction_heat = thermal.friction_heat_rate * normalized_rpm
	local total_heat = combustion_heat + friction_heat
	local retained_heat = total_heat * (1 - thermal.exhaust_heat_fraction)

	-- Distribute: 60% coolant, 40% oil
	local coolant_heat = retained_heat * 0.6
	local oil_heat = retained_heat * 0.4
	local safe_dt = math.max(dt, 0)

	-- Temperature update
	thermal.coolant_temp += coolant_heat
		/ (math.max(thermal.coolant_capacity, 0.1) * math.max(thermal.coolant_specific_heat, 1))
		* safe_dt
	thermal.oil_temp += oil_heat
		/ (math.max(thermal.oil_capacity, 0.1) * math.max(thermal.oil_specific_heat, 1))
		* safe_dt

	-- Cooling
	local cool_rate = EngineThermal.coolingRate(cooling, thermal.coolant_temp, vehicle_speed)
	local ambient_delta = math.max(thermal.coolant_temp - thermal.ambient_temp, 0)
	thermal.coolant_temp -= cool_rate
		/ (math.max(thermal.coolant_capacity, 0.1) * math.max(thermal.coolant_specific_heat, 1))
		* ambient_delta
		* safe_dt

	-- Heat soak toward ambient when engine is OFF (RPM = 0).
	-- At idle the engine produces enough friction heat to warm up,
	-- so soak must not fight the warmup phase.
	if (rpm or 0) <= 0 then
		local soak_rate = 0.01
		thermal.coolant_temp += (thermal.ambient_temp - thermal.coolant_temp) * soak_rate * safe_dt
		thermal.oil_temp += (thermal.ambient_temp - thermal.oil_temp) * soak_rate * safe_dt
	end

	-- Oil temp approaches coolant temp
	thermal.oil_temp += (thermal.coolant_temp - thermal.oil_temp) * 0.001 * safe_dt

	-- Transmission temp follows oil temp loosely
	thermal.transmission_temp += (thermal.oil_temp - thermal.transmission_temp) * 0.0005 * safe_dt

	-- Clamp temps
	thermal.coolant_temp = math.max(thermal.coolant_temp, thermal.ambient_temp - 5)
	thermal.oil_temp = math.max(thermal.oil_temp, thermal.ambient_temp - 5)

	-- Lubrication
	EngineThermal.updateLubrication(lube, accel_g, dt, rpm)

	-- Damage checks
	EngineThermal.checkOverheat(damage, thermal.coolant_temp)
	EngineThermal.checkOverrev(damage, rpm or 0, safe_redline)
	if (rpm or 0) > 0 then
		EngineThermal.checkOilStarvation(damage, lube.oil_pressure)
	end
	EngineThermal.updateDamage(damage, dt)

	-- Derate factor
	local derate_factor
	if damage.is_seized then
		derate_factor = 0
	else
		local bearing_derate = math.max(1 - damage.bearing_damage * 0.5, 0)
		local wear_derate = math.max(1 - damage.wear_level * 0.3, 0)
		local gasket_derate = if damage.head_gasket_failed then 0.6 else 1
		derate_factor = bearing_derate * wear_derate * gasket_derate
	end

	-- Stress accumulators
	EngineThermal.accumulateThermal(stress, thermal.coolant_temp, 110, dt)
	if (rpm or 0) > 0 then
		EngineThermal.accumulateOil(stress, lube.oil_pressure, lube.nominal_pressure, dt)
	else
		stress.oil_stress = math.max(stress.oil_stress - stress.oil_cooldown_rate * math.max(dt, 0), 0)
	end
	EngineThermal.accumulateOverrev(stress, rpm or 0, safe_redline, dt)
	EngineThermal.accumulateLugging(stress, rpm or 0, safe_throttle, gear or 1, dt)

	EngineThermal.checkLugging(damage, rpm or 0, safe_throttle, gear or 1, 0)

	local blowup_stage = EngineStressBlowupStage(stress)
	local stress_derate = EngineStressDerateFactor(stress)
	derate_factor = derate_factor * stress_derate

	-- Warning
	local engine_running = (rpm or 0) > 0
	local has_warning = thermal.coolant_temp > 105
		or (engine_running and lube.oil_pressure < 80)
		or damage.bearing_damage > 0.1
		or damage.is_on_fire
		or blowup_stage >= 1

	return {
		coolant_temp = thermal.coolant_temp,
		oil_temp = thermal.oil_temp,
		oil_pressure = lube.oil_pressure,
		transmission_temp = thermal.transmission_temp,
		derate_factor = derate_factor,
		has_warning = has_warning,
		is_seized = damage.is_seized,
		is_on_fire = damage.is_on_fire,
		blowup_stage = blowup_stage,
		thermal_stress = stress.thermal_stress,
		oil_stress = stress.oil_stress,
		overrev_stress = stress.overrev_stress,
		lugging_stress = stress.lugging_stress,
		fan_active = cooling.fan_active,
		fan_speed = cooling.fan_speed,
	}
end

-- ============================================================
-- Full vehicle state constructor
-- ============================================================

function EngineThermal.new(overrides)
	local o = overrides or {}
	return {
		thermal = EngineThermal.newThermal(o.thermal),
		cooling = EngineThermal.newCoolingSystem(o.cooling),
		lubrication = EngineThermal.newLubricationSystem(o.lubrication),
		damage = EngineThermal.newDamage(o.damage),
		stress = EngineThermal.newStressAccumulators(o.stress),
	}
end

return EngineThermal
