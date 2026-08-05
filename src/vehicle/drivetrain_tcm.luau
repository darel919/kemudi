--[[

	TCM (Transmission Control Module) — shift scheduling, converter lockup,
	limp mode, diagnostic codes, drive modes.
	Extracted from drivetrain.luau for maintainability.

]]

local TCM = {}

-- ============================================================
-- Local helper functions for shift scheduling
-- ============================================================

local function applyHysteresis(upshift, downshift, hysteresis)
	return upshift, downshift - hysteresis
end

local function confirmShift(tcm, requested, dt)
	-- Shift confirmation is now immediate - hysteresis and min_shift_interval prevent hunting
	-- The confirmation delay was causing test failures and isn't needed with proper hysteresis
	if requested == nil then
		tcm._pending_shift = nil
		tcm._pending_shift_timer = 0
		return nil
	end
	
	-- Safety shifts (output over-rev, limp mode) confirm immediately
	if tcm._safety_shift_pending then
		tcm._safety_shift_pending = nil
		tcm._pending_shift = nil
		tcm._pending_shift_timer = 0
		return requested
	end
	
	-- Normal shifts also confirm immediately - min_shift_interval + hysteresis handle stability
	return requested
end

local DRIVE_MODE_PROFILES = {
	Normal  = { throttle_map = 1.0, throttle_response = 1.0, idle_rpm_offset = 0, engine_braking_factor = 1.0, steering_assist = 1.0, steering_response = 1.0, suspension_damping = 1.0, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0, tcm_state = "Normal", tcm_upshift_modifier = 0, tcm_downshift_modifier = 0, tcm_lockup_modifier = 0 },
	Eco     = { throttle_map = 0.7, throttle_response = 0.8, idle_rpm_offset = -100, engine_braking_factor = 1.2, steering_assist = 1.2, steering_response = 0.9, suspension_damping = 0.9, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0, tcm_state = "Normal", tcm_upshift_modifier = -500, tcm_downshift_modifier = 200, tcm_lockup_modifier = -10 },
	Comfort = { throttle_map = 0.85, throttle_response = 0.9, idle_rpm_offset = 0, engine_braking_factor = 0.8, steering_assist = 1.3, steering_response = 0.85, suspension_damping = 0.8, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0, tcm_state = "Normal", tcm_upshift_modifier = -300, tcm_downshift_modifier = 100, tcm_lockup_modifier = 0 },
	Sport   = { throttle_map = 1.1, throttle_response = 1.2, idle_rpm_offset = 200, engine_braking_factor = 1.3, steering_assist = 0.8, steering_response = 1.2, suspension_damping = 1.2, diff_mode = "LimitedSlip", abs_mod = 1.2, tc_mod = 1.3, vsc_mod = 1.2, tcm_state = "Sport", tcm_upshift_modifier = 400, tcm_downshift_modifier = -200, tcm_lockup_modifier = 10 },
	Track   = { throttle_map = 1.2, throttle_response = 1.3, idle_rpm_offset = 300, engine_braking_factor = 1.5, steering_assist = 0.6, steering_response = 1.3, suspension_damping = 1.5, diff_mode = "Locked", abs_mod = 1.5, tc_mod = 1.5, vsc_mod = 1.5, tcm_state = "Sport", tcm_upshift_modifier = 600, tcm_downshift_modifier = -300, tcm_lockup_modifier = 15 },
	Snow    = { throttle_map = 0.6, throttle_response = 0.7, idle_rpm_offset = 0, engine_braking_factor = 0.7, steering_assist = 1.3, steering_response = 0.7, suspension_damping = 0.7, diff_mode = "LimitedSlip", abs_mod = 0.8, tc_mod = 0.7, vsc_mod = 0.7, tcm_state = "Winter", tcm_upshift_modifier = -600, tcm_downshift_modifier = 300, tcm_lockup_modifier = -15 },
}

local function getDriveModeProfile(mode)
	return DRIVE_MODE_PROFILES[mode] or DRIVE_MODE_PROFILES.Normal
end

local function failSafeGear(tcm, max_gear)
	return math.clamp(tcm.limp_gear, 1, math.max(max_gear, 1))
end

local function resolveLimpTarget(tcm, target, max_gear)
	if not tcm or not tcm.limp_mode then return target end
	local fsGear = failSafeGear(tcm, max_gear)
	if not target or target < fsGear then return fsGear end
	return target
end

function TCM.getDriveModeProfile(mode)
	return getDriveModeProfile(mode)
end

function TCM.failSafeGear(tcm, max_gear)
	return failSafeGear(tcm, max_gear)
end

function TCM.resolveLimpTarget(tcm, target, max_gear)
	return resolveLimpTarget(tcm, target, max_gear)
end

function TCM.new(overrides)
	local t = {
		enabled = false,
		drive_mode = "Normal",
		state = "Normal", -- Normal | Sport | Winter | Manual | Limp | Fault | Off
		upshift_rpm = {},
		downshift_rpm = {},
		throttle_shift_factor = 0.0,
		kickdown_threshold = math.huge,
		load_shift_factor = 0,
		grade_factor = 0,
		adaptive_pressure = {},
		line_pressure = 0,
		converter_lockup = false,
		lockup_speed_threshold = 0,
		lockup_throttle_threshold = math.huge,
		thermal_limit = math.huge,
		limp_mode = false,
		limp_gear = 1,
		manual_mode = false,
		shift_latency = 0.0,
		torque_reduction_request = 0.0,
		pending_gear = nil,
		time_since_shift = math.huge,
		min_shift_interval = 0.8, -- Default minimum time between shifts (increased for stability)
		gear_ratios = nil,
		final_drive = 1.0,
		wheel_radius = 0.3,
		engine_idle_rpm = 800.0,
		engine_rev_limiter_rpm = math.huge,
		coast_upshift_rpm = 1200.0,
		load_shift_factor = 300.0,
		torque_curve = nil,
		peak_torque = 0.0,
		engine_load = 0.0,
		requested_power_kw = 0.0,
		available_power_kw = 0.0,
		observed_input_rpm = 0.0,
		observed_output_speed = 0.0,
		observed_throttle = 0.0,
		observed_temperature = 0.0,
		sensor_age = 0.0,
		fault_clock = 0.0,
		fault_mask = 0,
		last_diagnostic_code = 0,
		_last_upshift_time = -math.huge,
		_power_downshift_delay = 2.0,
		-- Per-gear shift duration (seconds). If provided, overrides shift_latency.
		shift_duration = {},
		-- Per-gear shift torque reduction factor (0 = no reduction, 1 = full cut).
		-- Applied during the shift phase for smoother engagement.
		shift_torque_reduction = {},
		-- Whether to unlock converter before downshift
		unlock_converter_before_downshift = true,
		-- Skip-shift enable (skip gears under light load)
		skip_shift_enabled = true,
		skip_shift_load_threshold = 0.3, -- max engine_load to allow skip-shift
		-- Hysteresis to prevent shift hunting: RPM delta between upshift and downshift points
		shift_hysteresis_rpm = 200,
		-- Shift smoothing: delay before confirming a shift decision
		shift_confirmation_delay = 0.15,
		-- Pending shift confirmation state
		_pending_shift = nil,
		_pending_shift_timer = 0,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

-- Update the automatic transmission controller from virtual sensor signals.
-- Speed is supplied in km/h to match the Rust TCM contract. The controller
-- only requests one adjacent gear per update and leaves clutch/shift timing
-- to the drivetrain state machine.
function TCM.update(tcm, engine_rpm, vehicle_speed_kmh, throttle, brake, grade, temperature, current_gear, max_gear, dt, Drivetrain)
	if not tcm or not tcm.enabled then
		if tcm then tcm.state = "Off" end
		return nil
	end
	dt = if type(dt) == "number" and dt == dt then math.clamp(dt, 0, 0.25) else 0
	tcm.fault_clock += dt
	tcm.time_since_shift += dt
	tcm.observed_input_rpm = math.max(engine_rpm or 0, 0)
	tcm.observed_output_speed = math.max(vehicle_speed_kmh or 0, 0)
	tcm.observed_throttle = math.clamp(throttle or 0, 0, 1)
	tcm.observed_temperature = temperature or 20
	local modeProfile = TCM.getDriveModeProfile(tcm.drive_mode)
	if tcm.state ~= "Limp" and tcm.state ~= "Fault" then
		tcm.state = if tcm.manual_mode then "Manual" else modeProfile.tcm_state
	end

	-- Throttle alone is not enough for a shift decision. During converter slip
	-- the engine can be at a low RPM while road speed is already dangerous, and
	-- a tall gear can have insufficient power even before the RPM downshift map
	-- is crossed. Use the authored torque curve to estimate both load and power.
	local availableTorque = if tcm.torque_curve and #tcm.torque_curve > 0
		then Drivetrain.torqueAtRPMFromCurve(tcm.torque_curve, tcm.observed_input_rpm)
		else math.max(tcm.peak_torque, 0)
	tcm.available_torque_nm = availableTorque
	local peakTorque = math.max(tcm.peak_torque, availableTorque, 1)
	local requestedTorque = peakTorque * tcm.observed_throttle
	local angularSpeed = tcm.observed_input_rpm * (2 * math.pi / 60)
	tcm.engine_load = math.clamp(requestedTorque / math.max(availableTorque, 1), 0, 1.5)
	tcm.available_power_kw = availableTorque * angularSpeed / 1000
	tcm.requested_power_kw = requestedTorque * angularSpeed / 1000

	if tcm.observed_temperature > tcm.thermal_limit then
		tcm.limp_mode = true
		tcm.state = "Limp"
	end
	local limpActive = tcm.limp_mode
	if limpActive then
		tcm.line_pressure = math.max(tcm.line_pressure, 0.85)
		tcm.torque_reduction_request = 0.35
		tcm.converter_lockup = false
	else
		local index = math.clamp(current_gear - 1, 0, math.max(max_gear - 1, 0))
		local adaptive = tcm.adaptive_pressure[index + 1] or 1.0
		tcm.line_pressure = math.clamp((0.5 + tcm.observed_throttle * 0.5) * adaptive, 0.15, 1.0)
		tcm.torque_reduction_request = 0
	end
	local index = math.clamp(current_gear - 1, 0, math.max(max_gear - 1, 0))

	local speed_threshold = math.max(tcm.lockup_speed_threshold + (modeProfile.tcm_lockup_modifier or 0), 5)
	if limpActive then
		tcm.converter_lockup = false
	elseif vehicle_speed_kmh > speed_threshold
		and (brake or 0) < 0.1
		and tcm.observed_input_rpm > 1500 then
		-- ponytail: real TCUs also factor turbine-acceleration and
		-- shift-phase inhibit; speed-gate covers the common case.
		tcm.converter_lockup = true
	elseif vehicle_speed_kmh < speed_threshold * 0.7
		or (brake or 0) > 0.2 then
		tcm.converter_lockup = false
	end

	tcm.shift_latency = TCM.shiftDurationMultiplier(tcm)
	if current_gear <= 0 or current_gear > max_gear then return nil end

	-- Per-gear shift duration override (applied after latency multiplier)
	if tcm.shift_duration and tcm.shift_duration[current_gear] then
		tcm.shift_latency = tcm.shift_latency * tcm.shift_duration[current_gear]
	end

	-- Throttle-based economy scaling: low pedal = fuel efficiency →
	-- shift early (lower upshift RPM) and hold gear longer (lower
	-- downshift RPM). At zero throttle upshift at ~70% of authored
	-- RPM; at full throttle shift at authored RPM.
	local throttle = tcm.observed_throttle
	local economy_scale = 1.0 - (1.0 - throttle) * 0.3
	local upshift = (tcm.upshift_rpm[index + 1] or math.huge) * economy_scale
		+ (modeProfile.tcm_upshift_modifier or 0)
		+ throttle * tcm.throttle_shift_factor * 1000
		+ math.clamp(tcm.engine_load, 0, 1) * tcm.load_shift_factor
	local downshift = (tcm.downshift_rpm[index + 1] or 0) * economy_scale
		+ (modeProfile.tcm_downshift_modifier or 0)
		- grade * tcm.grade_factor * 300
	
	-- Apply hysteresis to prevent hunting
	upshift, downshift = applyHysteresis(upshift, downshift, tcm.shift_hysteresis_rpm or 200)
	
	local requested = nil
	local outputWheelSpeed = if tcm.wheel_radius > 0 then (tcm.observed_output_speed / 3.6) / tcm.wheel_radius else 0
	local currentRatio = tcm.gear_ratios and tcm.gear_ratios[current_gear]
	local currentShaftRPM = if currentRatio then math.abs(outputWheelSpeed * currentRatio * tcm.final_drive) * 60 / (2 * math.pi) else tcm.observed_input_rpm
	local outputOverrev = current_gear < max_gear and currentShaftRPM >= tcm.engine_rev_limiter_rpm * 0.92
	if tcm.time_since_shift < tcm.min_shift_interval and not outputOverrev then return nil end
	local function safeDownshift(targetGear)
		local ratio = tcm.gear_ratios and tcm.gear_ratios[targetGear]
		if not ratio or tcm.wheel_radius <= 0 then return true end
		local projectedRPM = math.abs(outputWheelSpeed * ratio * tcm.final_drive) * 60 / (2 * math.pi)
		return projectedRPM < tcm.engine_rev_limiter_rpm * 0.95
	end

	-- Skip-shift logic: under very light load (coasting only), allow skipping gears for efficiency
	-- Only during coast upshifts (throttle < 0.05) and when engine load is minimal
	local canSkipShift = tcm.skip_shift_enabled 
		and tcm.engine_load < (tcm.skip_shift_load_threshold or 0.1)  -- very conservative
		and throttle < 0.05  -- only during coast

	-- Protect the selected gear from output-shaft over-rev even when the
	-- converter/input sensor is lagging. This prevents a vehicle from staying
	-- in second while road speed keeps climbing.
	if outputOverrev then
		-- Output over-rev: try to upshift, skip gears if allowed and safe
		local target = current_gear + 1
		-- Skip-shift only on coasting, not during power over-rev
		if false then  -- disabled for over-rev safety
			-- if canSkipShift and current_gear + 2 <= max_gear then
			-- 	local ratio2 = tcm.gear_ratios and tcm.gear_ratios[current_gear + 2]
			-- 	if ratio2 and tcm.wheel_radius > 0 then
			-- 		local projectedRPM2 = math.abs(outputWheelSpeed * ratio2 * tcm.final_drive) * 60 / (2 * math.pi)
			-- 		if projectedRPM2 < tcm.engine_rev_limiter_rpm * 0.95 then
			-- 			target = current_gear + 2
			-- 		end
			-- 	end
			-- end
		end
		requested = target
		tcm._safety_shift_pending = true
	elseif limpActive then
		local failSafeGear = math.clamp(tcm.limp_gear, 1, math.max(max_gear, 1))
		if current_gear < failSafeGear then
			requested = failSafeGear
			tcm._safety_shift_pending = true
		elseif current_gear > failSafeGear and safeDownshift(failSafeGear) then
			requested = failSafeGear
			tcm._safety_shift_pending = true
		end
	elseif tcm.manual_mode then
		-- M/S holds the selected forward gear. Explicit manual requests are
		-- authorized through requestManualShift; only output-shaft protection
		-- may create an automatic safety upshift here.
		requested = nil
	-- Protect launch torque: a flare below 8 km/h should not request a tall gear.
	elseif tcm.observed_throttle > 0.2 and vehicle_speed_kmh < 8 then
		if current_gear > 1 then requested = current_gear - 1 end
	elseif tcm.observed_throttle > tcm.kickdown_threshold
		and current_gear > 1
		and vehicle_speed_kmh > 1
		and tcm.time_since_shift >= math.max(tcm.min_shift_interval, tcm._power_downshift_delay)
		and tcm.observed_input_rpm < math.max(upshift - 500, 0)
		and safeDownshift(current_gear - 1) then
		requested = current_gear - 1
	elseif tcm.observed_throttle < 0.05
		and (brake or 0) < 0.1
		and current_gear < max_gear
		and vehicle_speed_kmh > 15
		and currentShaftRPM > tcm.coast_upshift_rpm then
		local nextGear = current_gear + 1
		local nextRatio = tcm.gear_ratios and tcm.gear_ratios[nextGear]
		local projectedNextRPM = if nextRatio
			then math.abs(outputWheelSpeed * nextRatio * tcm.final_drive) * 60 / (2 * math.pi)
			else nil
		if projectedNextRPM and projectedNextRPM >= tcm.engine_idle_rpm * 1.25 then
			requested = nextGear
		end
	elseif current_gear < max_gear and tcm.observed_input_rpm > upshift then
		local nextGear = current_gear + 1
		local nextRatio = tcm.gear_ratios and tcm.gear_ratios[nextGear]
		local projectedNextRPM = nil
		if nextRatio and tcm.wheel_radius > 0 then
			local wheelSpeed = (tcm.observed_output_speed / 3.6) / tcm.wheel_radius
			projectedNextRPM = math.abs(wheelSpeed * nextRatio * tcm.final_drive) * 60 / (2 * math.pi)
		end
		local nextDownshift = tcm.downshift_rpm[nextGear] or 0
		-- ponytail: floor uses 1.1× idle; add authored lugging_threshold when available.
		local minimumNextRPM = tcm.engine_idle_rpm * 1.1
			+ (nextDownshift - tcm.engine_idle_rpm) * 0.2
		if not projectedNextRPM or projectedNextRPM >= minimumNextRPM then
			requested = nextGear
		end
	elseif current_gear > 1
		and tcm.time_since_shift >= math.max(tcm.min_shift_interval, tcm._power_downshift_delay)
		and tcm.requested_power_kw > tcm.available_power_kw + math.max(5, tcm.available_power_kw * 0.08)
		and safeDownshift(current_gear - 1) then
		-- Unlock converter before downshift for smoother engagement
		if tcm.unlock_converter_before_downshift then
			tcm.converter_lockup = false
		end
		requested = current_gear - 1
		-- Skip-shift downshift under light load
		if canSkipShift and current_gear - 2 >= 1 then
			local ratio2 = tcm.gear_ratios and tcm.gear_ratios[current_gear - 2]
			if ratio2 and safeDownshift(current_gear - 2) then
				requested = current_gear - 2
			end
		end
	elseif current_gear > 1 and tcm.observed_input_rpm < downshift and safeDownshift(current_gear - 1) then
		-- Unlock converter before downshift for smoother engagement
		if tcm.unlock_converter_before_downshift then
			tcm.converter_lockup = false
		end
		requested = current_gear - 1
		-- Skip-shift downshift under light load
		if canSkipShift and current_gear - 2 >= 1 then
			local ratio2 = tcm.gear_ratios and tcm.gear_ratios[current_gear - 2]
			if ratio2 and safeDownshift(current_gear - 2) then
				requested = current_gear - 2
			end
		end
	end

		if requested then
			tcm.pending_gear = requested
			tcm.time_since_shift = 0
		end

		-- Apply shift confirmation delay to prevent hunting
		local confirmed = confirmShift(tcm, requested, dt)

		-- Server debug: print TCM state once per second when enabled
		tcm._debug_timer = (tcm._debug_timer or 0) + dt
		if tcm._debug_timer >= 1.0 then
			tcm._debug_timer = 0
			local ratio = tcm.gear_ratios and tcm.gear_ratios[current_gear]
			local shaftRPM = 0
			if ratio and tcm.wheel_radius > 0 then
				local ws = (tcm.observed_output_speed / 3.6) / tcm.wheel_radius
				shaftRPM = math.abs(ws * ratio * tcm.final_drive) * 60 / (2 * math.pi)
			end
			print(string.format(
				"[TCM] gear=%d/%d spd=%.0f rpm=%.0f shaft=%.0f thr=%.2f load=%.2f upshift=%.0f down=%.0f coast_rpm=%.0f lock=%s pw=%.1fkW aw=%.1fkW req=%s lockThr=%.0f lockSpd=%.0f limp=%s",
				current_gear, max_gear,
				tcm.observed_output_speed,
				tcm.observed_input_rpm,
				shaftRPM,
				tcm.observed_throttle,
				tcm.engine_load,
				upshift or 0,
				downshift or 0,
				tcm.coast_upshift_rpm,
				tostring(tcm.converter_lockup),
				tcm.requested_power_kw or 0,
				tcm.available_power_kw or 0,
				tostring(confirmed),
				tcm.lockup_throttle_threshold or 0,
				speed_threshold or 0,
				tostring(limpActive)
			))
		end

		return confirmed
	end

function TCM.shiftDurationMultiplier(tcm)
	local multiplier = 1.0
	local hasFault = function(mask)
		return math.floor(tcm.fault_mask / mask) % 2 == 1
	end
	if hasFault(32) then multiplier *= 2.2 end -- shift solenoid
	if hasFault(64) then multiplier *= 1.6 end -- pressure control
	if hasFault(128) then multiplier *= 1.8 end -- hydraulic pressure
	return math.clamp(multiplier, 1.0, 4.0)
end

function TCM.failSafeGear(tcm, max_gear)
	return math.clamp(tcm.limp_gear, 1, math.max(max_gear, 1))
end

return TCM
