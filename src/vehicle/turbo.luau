--[[
	Turbo / Supercharger simulation.
	Pure Luau, no Roblox API dependencies.
	Spool lag, wastegate, BOV, anti-lag, boost leak, intercooler.

	ALL CONFIG MUST BE PROVIDED BY VEHICLE ASSET.
	No vehicle-specific defaults in engine code.
--]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")

local Turbo = {}

-- ============================================================
-- Config — REQUIRED from vehicle asset, no defaults
-- ============================================================

function Turbo.newConfig(overrides)
	overrides = overrides or {}
	-- Engine code provides NO vehicle-specific defaults.
	-- If a field is missing, it means the vehicle asset didn't provide it.
	-- The engine must fail loudly or behave as "no effect" (nil/0/1).
	return {
		-- Required: "Turbo" | "Supercharger" | "None"
		type = overrides.type,

		-- Turbo parameters (required if type == "Turbo")
		max_boost_psi = overrides.max_boost_psi,
		spool_time = overrides.spool_time,
		wastegate_open_psi = overrides.wastegate_open_psi,
		wastegate_duty_cycle = overrides.wastegate_duty_cycle,
		bov_threshold = overrides.bov_threshold,
		bov_vent_rate = overrides.bov_vent_rate,
		intercooler_efficiency = overrides.intercooler_efficiency,
		intercooler_pressure_loss = overrides.intercooler_pressure_loss,
		compressor_efficiency = overrides.compressor_efficiency,
		anti_lag_enabled = overrides.anti_lag_enabled,
		anti_lag_spool_rate = overrides.anti_lag_spool_rate,
		anti_lag_exhaust_temp_rise = overrides.anti_lag_exhaust_temp_rise,
		boost_leak_rate = overrides.boost_leak_rate,
		boost_leak_pressure_loss = overrides.boost_leak_pressure_loss,
		exhaust_energy_coeff = overrides.exhaust_energy_coeff,
		torque_per_psi = overrides.torque_per_psi,
		reference_charge_temp = overrides.reference_charge_temp,

		-- Supercharger parameters (required if type == "Supercharger")
		sc_efficiency = overrides.sc_efficiency,
		sc_parasitic_loss = overrides.sc_parasitic_loss,
		sc_boost_at_rpm = overrides.sc_boost_at_rpm,
		sc_torque_per_psi = overrides.sc_torque_per_psi,
	}
end

-- ============================================================
-- State
-- ============================================================

function Turbo.newState()
	return {
		boost_pressure = 0.0,
		spool_position = 0.0,
		wastegate_position = 0.0,
		bov_position = 0.0,
		charge_air_temp = 25.0,
		exhaust_temp = 200.0,
		compressor_outlet_temp = 80.0,
		is_boosting = false,
		anti_lag_active = false,
	}
end

-- ============================================================
-- Internal helpers
-- ============================================================

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

local function nan_safe(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end -- NaN
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

--- Linearly interpolate a { {rpm, value}, ... } curve.
local function interpCurve(curve, rpm)
	if not curve or #curve == 0 then return 0 end
	if rpm <= curve[1][1] then return curve[1][2] end
	if rpm >= curve[#curve][1] then return curve[#curve][2] end
	for i = 1, #curve - 1 do
		local r0, v0 = curve[i][1], curve[i][2]
		local r1, v1 = curve[i + 1][1], curve[i + 1][2]
		if rpm >= r0 and rpm <= r1 then
			local t = (rpm - r0) / math.max(r1 - r0, 1)
			return v0 + t * (v1 - v0)
		end
	end
	return curve[#curve][2]
end

-- ============================================================
-- Update (mutates state in-place)
-- ============================================================

function Turbo.update(state, config, rpm, throttle, ambient_temp, dt)
	-- NaN guards on inputs
	rpm = nan_safe(rpm, 0)
	throttle = clamp(nan_safe(throttle, 0), 0, 1)
	ambient_temp = nan_safe(ambient_temp, 25)
	dt = nan_safe(dt, 1/60)

	local t = config.type
	if not t or t == "None" then
		state.boost_pressure = 0
		state.spool_position = 0
		state.is_boosting = false
		return state
	end

	-- Common parameters with neutral defaults for "no effect" when missing
	local maxBoost = nan_safe(config.max_boost_psi, 0)
	local spoolTime = nan_safe(config.spool_time, 1) -- 1s = no spool lag effect
	local wastegateOpen = nan_safe(config.wastegate_open_psi, 0)
	local wastegateDuty = nan_safe(config.wastegate_duty_cycle, 0)
	local bovThreshold = nan_safe(config.bov_threshold, 0)
	local bovVentRate = nan_safe(config.bov_vent_rate, 0)
	local icEfficiency = nan_safe(config.intercooler_efficiency, 0)
	local icPressureLoss = nan_safe(config.intercooler_pressure_loss, 0)
	local compEfficiency = nan_safe(config.compressor_efficiency, 0)
	local antiLagEnabled = config.anti_lag_enabled == true
	local antiLagSpoolRate = nan_safe(config.anti_lag_spool_rate, 0)
	local antiLagTempRise = nan_safe(config.anti_lag_exhaust_temp_rise, 0)
	local boostLeakRate = nan_safe(config.boost_leak_rate, 0)
	local boostLeakLoss = nan_safe(config.boost_leak_pressure_loss, 0)
	local exhaustEnergyCoeff = nan_safe(config.exhaust_energy_coeff, 0)
	local antiLagDelta = 0

	local scEfficiency = nan_safe(config.sc_efficiency, 0)
	local scParasiticLoss = nan_safe(config.sc_parasitic_loss, 0)
	local scBoostCurve = config.sc_boost_at_rpm

	-- Target boost from throttle (0-1) * max boost
	local targetBoost = maxBoost * throttle

	if t == "Turbo" then
		-- Exhaust energy drives turbine
		local exhaustEnergy = exhaustEnergyCoeff * rpm * rpm * throttle
		exhaustEnergy = clamp(exhaustEnergy, 0, 1)

		-- Spool dynamics: first-order lag toward target
		-- tau = spool_time at full load, faster at high exhaust energy
		local effectiveTau = spoolTime / math.max(exhaustEnergy, 0.1)
		local spoolRate = 1.0 / math.max(effectiveTau, 1e-6) * dt

		-- Anti-lag: extra spool when off-throttle
		if antiLagEnabled and throttle < 0.1 then
			antiLagDelta = antiLagSpoolRate * dt
			state.anti_lag_active = true
			state.exhaust_temp = nan_safe(state.exhaust_temp, 200) + antiLagTempRise * dt
		else
			state.anti_lag_active = false
		end

		-- Spool position integrates toward target
		state.spool_position = clamp(state.spool_position + spoolRate * (targetBoost / math.max(maxBoost, 1) - state.spool_position) + antiLagDelta, 0, 1)

		-- Boost pressure = spool position * target
		local rawBoost = state.spool_position * targetBoost

		-- Wastegate: opens when boost exceeds threshold
		if wastegateOpen > 0 and rawBoost > wastegateOpen then
			local excess = rawBoost - wastegateOpen
			local wastegateOpenAmount = clamp(excess * wastegateDuty * 2, 0, 1)
			state.wastegate_position = wastegateOpenAmount
			rawBoost = rawBoost * (1 - wastegateOpenAmount * 0.8) -- wastegate vents excess
		else
			state.wastegate_position = 0
		end

		-- BOV (blow-off valve): vents on sudden throttle drop
		if bovThreshold > 0 and throttle < bovThreshold and state.boost_pressure > 0.5 then
			state.bov_position = clamp(state.bov_position + bovVentRate * dt, 0, 1)
			rawBoost = rawBoost * (1 - state.bov_position)
		else
			state.bov_position = clamp(state.bov_position - bovVentRate * dt * 0.5, 0, 1)
		end

		-- Boost leak
		if boostLeakRate > 0 then
			rawBoost = rawBoost * (1 - boostLeakRate * dt * boostLeakLoss)
		end

		state.boost_pressure = clamp(rawBoost, 0, maxBoost * 1.5)

		-- Compressor outlet temp (adiabatic compression + intercooler)
		-- T_out = T_in * (P_out/P_in)^((gamma-1)/gamma) / efficiency
		local pressureRatio = (state.boost_pressure / 14.7) + 1.0 -- absolute pressure ratio
		local gamma = 1.4 -- air
		local isentropicExp = (gamma - 1) / gamma
		local compOutletTempK = (ambient_temp + 273.15) * (pressureRatio ^ isentropicExp) / math.max(compEfficiency, 0.1)

		-- Intercooler: reduces temp toward ambient
		if icEfficiency > 0 then
			compOutletTempK = compOutletTempK - (compOutletTempK - (ambient_temp + 273.15)) * icEfficiency
			-- Pressure loss through intercooler
			state.boost_pressure = state.boost_pressure * (1 - icPressureLoss)
		end

		state.compressor_outlet_temp = compOutletTempK - 273.15
		state.charge_air_temp = state.compressor_outlet_temp
		state.is_boosting = state.boost_pressure > 0.5

	elseif t == "Supercharger" then
		-- Supercharger: boost directly proportional to RPM via curve
		local boostFraction = interpCurve(scBoostCurve, rpm)
		state.boost_pressure = clamp(targetBoost * boostFraction, 0, maxBoost * 1.5)

		-- Parasitic loss from belt drive
		-- Returned via getEffectiveTorque
		state.charge_air_temp = ambient_temp + 40 -- typical SC heat rise
		state.is_boosting = state.boost_pressure > 0.5
		state.spool_position = boostFraction
	end

	return state
end

-- Returns effective engine torque multiplier from boost
function Turbo.getEffectiveTorque(state, config, base_torque)
	base_torque = nan_safe(base_torque, 0)
	local t = config.type
	if not t or t == "None" then return base_torque end

	if t == "Turbo" then
		-- Torque gain proportional to boost pressure
		-- Vehicle asset must provide torque_per_psi; if missing, no torque effect
		local boostGain = assert(config.torque_per_psi, "turbo torque_per_psi was not authored")
		local referenceTemp = assert(config.reference_charge_temp, "turbo reference charge temperature was not authored")
		local chargeTemp = nan_safe(state.charge_air_temp, referenceTemp)
		local densityFactor = (referenceTemp + 273.15) / math.max(chargeTemp + 273.15, 1e-6)
		local boostPressure = nan_safe(state.boost_pressure, 0)
		return base_torque * (1 + boostPressure * boostGain) * densityFactor

	elseif t == "Supercharger" then
		-- SC: boost is linear with RPM, but parasitic loss reduces net
		local boostGain = assert(config.sc_torque_per_psi, "supercharger torque_per_psi was not authored")
		local parasitic = nan_safe(config.sc_parasitic_loss, 0) * base_torque
		return math.max(base_torque * (1 + state.boost_pressure * boostGain) - parasitic, 0)
	end

	return base_torque
end

function Turbo.telemetry(state)
	return {
		boost_psi = state.boost_pressure,
		spool = state.spool_position,
		wastegate = state.wastegate_position,
		bov = state.bov_position,
		charge_temp = state.charge_air_temp,
		exhaust_temp = state.exhaust_temp,
		compressor_outlet_temp = state.compressor_outlet_temp,
		is_boosting = state.is_boosting,
		anti_lag = state.anti_lag_active,
	}
end

return Turbo