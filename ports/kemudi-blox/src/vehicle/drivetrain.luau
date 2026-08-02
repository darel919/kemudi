--[[
	Drivetrain — engine, transmission, differential, TCM, drive modes.
	Ported from crates/kemudi-engine/src/drivetrain/*.
	Pure Luau, no Roblox API dependencies.
--]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")

local Drivetrain = {}

-- ============================================================
-- Engine
-- ============================================================

local DEFAULT_TORQUE_CURVE = {
	{ 0, 100 }, { 1000, 150 }, { 2000, 200 }, { 3000, 250 },
	{ 4000, 280 }, { 5000, 260 }, { 6000, 220 }, { 7000, 160 }, { 8000, 100 },
}

function Drivetrain.newEngine(overrides)
	local e = {
		rpm = 800.0,
		idle_rpm = 800.0,
		redline_rpm = 7000.0,
		rev_limiter_rpm = 7500.0,
		torque_curve = DEFAULT_TORQUE_CURVE,
		throttle_response = 0.8,
		engine_braking = 30.0,
	}
	if overrides then for k, v in overrides do e[k] = v end end
	return e
end

function Drivetrain.engineTorqueAtRPM(engine, rpm)
	local curve = engine.torque_curve
	if #curve == 0 then return 0 end
	if rpm <= curve[1][1] then return curve[1][2] end
	if rpm >= curve[#curve][1] then return curve[#curve][2] end
	for i = 1, #curve - 1 do
		local r0, t0 = curve[i][1], curve[i][2]
		local r1, t1 = curve[i+1][1], curve[i+1][2]
		if rpm >= r0 and rpm <= r1 then
			local t = (rpm - r0) / math.max(r1 - r0, 1)
			return t0 + t * (t1 - t0)
		end
	end
	return 0
end

function Drivetrain.updateEngine(engine, throttle, dt)
	local target_rpm
	if throttle > 0.01 then
		target_rpm = engine.idle_rpm + (engine.redline_rpm - engine.idle_rpm) * throttle
	else
		target_rpm = engine.idle_rpm
	end
	engine.rpm = engine.rpm + (target_rpm - engine.rpm) * (engine.throttle_response * 10.0) * dt
	engine.rpm = math.clamp(engine.rpm, 0, engine.rev_limiter_rpm)
end

-- ============================================================
-- Transmission
-- ============================================================

function Drivetrain.newTransmission(overrides)
	local t = {
		mode = "Manual", -- "Manual" | "Automatic"
		gear_ratios = { 3.5, 2.1, 1.4, 1.0, 0.7 },
		final_drive = 3.7,
		current_gear = 1,
		reverse_ratio = -3.2,
		clutch_engagement = 1.0,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

function Drivetrain.totalRatio(trans)
	local gear = trans.current_gear
	if gear >= 1 and gear <= #trans.gear_ratios then
		return trans.gear_ratios[gear] * trans.final_drive
	elseif gear == -1 then
		return trans.reverse_ratio * trans.final_drive
	end
	return 0
end

function Drivetrain.engineRPMFromWheelSpeed(trans, wheel_speed)
	local ratio = math.abs(Drivetrain.totalRatio(trans))
	return math.abs(wheel_speed) * ratio * 60 / (2 * math.pi)
end

function Drivetrain.wheelTorque(trans, engine_torque)
	return engine_torque * Drivetrain.totalRatio(trans) * trans.clutch_engagement
end

function Drivetrain.shiftUp(trans)
	if trans.current_gear >= #trans.gear_ratios then return false end
	trans.current_gear += 1
	return true
end

function Drivetrain.shiftDown(trans)
	if trans.current_gear <= -1 then return false end
	trans.current_gear -= 1
	return true
end

-- ============================================================
-- Differential
-- ============================================================

function Drivetrain.newDifferential(overrides)
	local d = {
		mode = "Open", -- "Open" | "Locked" | "LimitedSlip"
		bias = 0.5,
		left_ratio = 0.5,
		right_ratio = 0.5,
	}
	if overrides then for k, v in overrides do d[k] = v end end
	return d
end

function Drivetrain.applyDifferential(diff, torque, left_grip, right_grip, left_speed, right_speed)
	left_speed = left_speed or 0
	right_speed = right_speed or 0
	local total_grip = left_grip + right_grip
	local grip_share = if total_grip > 0.001 then left_grip / total_grip else 0.5

	local left_ratio
	if diff.mode == "Open" then
		left_ratio = 0.5
	elseif diff.mode == "Locked" then
		local speed_scale = math.max(math.abs(left_speed), math.abs(right_speed), 1.0)
		local speed_transfer = ((right_speed - left_speed) / speed_scale) * 0.25
		left_ratio = math.clamp(grip_share + speed_transfer, 0, 1)
	elseif diff.mode == "LimitedSlip" then
		local locking = math.clamp(diff.bias, 0, 1)
		left_ratio = math.clamp(0.5 * (1 - locking) + grip_share * locking, 0, 1)
	else
		left_ratio = 0.5
	end

	diff.left_ratio = left_ratio
	diff.right_ratio = 1.0 - left_ratio
	return torque * left_ratio, torque * diff.right_ratio
end

-- ============================================================
-- Shift phase and clutch shock
-- ============================================================

function Drivetrain.newShiftState()
	return {
		phase = "Idle", -- Idle | Disengaging | Neutral | Engaging
		timer = 0,
		duration = 0.15,
		duration_multiplier = 1.0,
		pending_gear = 0,
	}
end

function Drivetrain.newClutchShock()
	return { magnitude = 0.0, decay_rate = 8.0, torque_spike = 0.0 }
end

function Drivetrain.applyClutchShock(shock, rpm_mismatch, clutch_engagement, dt)
	local raw = math.min(math.abs(rpm_mismatch) / 1000.0, 1.0) * clutch_engagement
	local decay = math.max(0, 1 - math.max(0, shock.decay_rate) * math.max(0, dt))
	shock.magnitude = math.max(raw, shock.magnitude * decay)
	shock.torque_spike = raw * 200.0
end

-- ============================================================
-- Auto-shift logic
-- ============================================================

function Drivetrain.newAutoShift(overrides)
	local s = {
		upshift_rpm = 6000.0,
		downshift_rpm = 2500.0,
		shift_delay = 0.5,
		kickdown_threshold = 0.8,
		last_shift_time = 0.0,
	}
	if overrides then for k, v in overrides do s[k] = v end end
	return s
end

function Drivetrain.autoShouldUpshift(auto, current_gear, rpm, throttle, time)
	if time - auto.last_shift_time < auto.shift_delay then return false end
	local effective = if throttle > auto.kickdown_threshold then auto.upshift_rpm - 500 else auto.upshift_rpm
	return rpm >= effective
end

function Drivetrain.autoShouldDownshift(auto, current_gear, rpm, time)
	if time - auto.last_shift_time < auto.shift_delay then return false end
	return rpm <= auto.downshift_rpm
end

function Drivetrain.autoRecordShift(auto, time)
	auto.last_shift_time = time
end

-- ============================================================
-- Drivetrain core (combined aggregator)
-- ============================================================

function Drivetrain.new(overrides)
	local d = {
		engine = Drivetrain.newEngine(),
		transmission = Drivetrain.newTransmission(),
		differential = Drivetrain.newDifferential(),
		wheel_speed = 0.0,
		throttle_input = 0.0,
		brake_input = 0.0,
		clutch_input = 0.0,
		shift = Drivetrain.newShiftState(),
		shift_duration = 0.15,
		auto_shift = Drivetrain.newAutoShift(),
		time = 0.0,
		is_stalled = false,
		last_drive_torque = 0.0,
		converter_coupling = 1.0,
		converter_torque_multiplier = 1.0,
		wheel_radius = 0.3,
		wheel_inertia = 0.0,
		max_downshift_rpm = 8000.0,
		clutch_shock = Drivetrain.newClutchShock(),
		overrev_events = {},
	}
	if overrides then for k, v in overrides do d[k] = v end end
	return d
end

function Drivetrain.effectiveWheelInertia(dt, vehicle_mass, wheel_inertia)
	if wheel_inertia > 0 then return wheel_inertia end
	return math.max(vehicle_mass * 0.01, 0.1)
end

function Drivetrain.update(dt, vehicle_mass, d)
	d.time += dt
	d.throttle_input = math.clamp(d.throttle_input, 0, 1)
	d.brake_input = math.clamp(d.brake_input, 0, 1)
	d.clutch_input = math.clamp(d.clutch_input, 0, 1)

	-- Shift phase update
	local shift = d.shift
	if shift.phase == "Disengaging" then
		shift.timer -= dt
		if shift.timer <= 0 then
			d.transmission.current_gear = shift.pending_gear
			shift.phase = "Neutral"
			shift.timer = shift.duration / 3.0
		end
	elseif shift.phase == "Neutral" then
		shift.timer -= dt
		if shift.timer <= 0 then
			shift.phase = "Engaging"
			shift.timer = shift.duration / 3.0
		end
	elseif shift.phase == "Engaging" then
		shift.timer -= dt
		Drivetrain.applyClutchShock(d.clutch_shock,
			d.engine.rpm - Drivetrain.engineRPMFromWheelSpeed(d.transmission, d.wheel_speed),
			d.transmission.clutch_engagement, dt)
		if shift.timer <= 0 then
			shift.phase = "Idle"
			Drivetrain.autoRecordShift(d.auto_shift, d.time)
		end
	end

	-- Clutch engagement
	d.transmission.clutch_engagement = if shift.phase == "Neutral" then 0
		elseif d.transmission.mode == "Automatic" then 1.0
		else 1.0 - d.clutch_input

	-- Rigid coupling detection
	local coupled_rpm = Drivetrain.engineRPMFromWheelSpeed(d.transmission, d.wheel_speed)
	local rigid_coupling = d.transmission.clutch_engagement > 0.95
		and (d.transmission.mode == "Manual" or d.converter_coupling > 0.95)
		and (math.abs(d.wheel_speed) > 0.5 or d.engine.rpm >= d.engine.idle_rpm * 0.7)

	-- Engine torque
	local engine_torque
	if rigid_coupling and coupled_rpm >= d.engine.rev_limiter_rpm then
		engine_torque = 0
	else
		engine_torque = Drivetrain.engineTorqueAtRPM(d.engine, d.engine.rpm) * d.throttle_input
	end

	-- Wheel torque through drivetrain
	local converter_factor = if d.transmission.mode == "Automatic"
		then math.clamp(d.converter_coupling * d.converter_torque_multiplier, 0, 1.2)
		else 1.0
	local drive_torque = Drivetrain.wheelTorque(d.transmission, engine_torque) * converter_factor

	-- Angular integration
	local wheel_inertia = Drivetrain.effectiveWheelInertia(dt, vehicle_mass, d.wheel_inertia)
	local stopping_torque = math.abs(d.wheel_speed) * wheel_inertia / math.max(dt, 1e-6)

	-- Engine braking
	local requested_engine_brake = 0
	if d.throttle_input < 0.05 and math.abs(d.wheel_speed) > 1e-6 and d.transmission.clutch_engagement > 0.05 then
		requested_engine_brake = d.engine.engine_braking * math.abs(Drivetrain.totalRatio(d.transmission))
	end

	-- Brake
	local direction = if math.abs(d.wheel_speed) > 1e-6 then Math.sign(d.wheel_speed) else 1.0
	local requested_brake = d.brake_input * 500.0
	local resistive_torque = math.min(requested_engine_brake + requested_brake, stopping_torque)
	local engine_brake_share = if requested_engine_brake + requested_brake > 1e-9
		then resistive_torque * requested_engine_brake / (requested_engine_brake + requested_brake)
		else 0
	local engine_brake_torque = engine_brake_share * direction
	local brake_torque = (resistive_torque - engine_brake_share) * direction

	d.last_drive_torque = drive_torque - engine_brake_torque
	local shaft_torque = d.last_drive_torque - brake_torque
	local angular_accel = shaft_torque / (wheel_inertia + 1e-6)
	d.wheel_speed = math.clamp(d.wheel_speed + angular_accel * dt, -200, 200)

	-- RPM coupling
	if shift.phase == "Neutral" then
		Drivetrain.updateEngine(d.engine, d.throttle_input, dt)
	elseif rigid_coupling then
		d.engine.rpm = math.clamp(math.max(coupled_rpm, d.engine.idle_rpm), 0, d.engine.rev_limiter_rpm)
	elseif d.transmission.clutch_engagement > 0.5 and (d.transmission.mode ~= "Automatic" or d.converter_coupling > 0.95) then
		local target = Drivetrain.engineRPMFromWheelSpeed(d.transmission, d.wheel_speed)
		local blend = d.transmission.clutch_engagement * d.converter_coupling * d.engine.throttle_response
		d.engine.rpm += (math.max(target, d.engine.idle_rpm) - d.engine.rpm) * blend * dt
		d.engine.rpm = math.min(d.engine.rpm, d.engine.rev_limiter_rpm)
	elseif d.transmission.mode == "Automatic" then
		Drivetrain.updateEngine(d.engine, d.throttle_input, dt)
		local target = Drivetrain.engineRPMFromWheelSpeed(d.transmission, d.wheel_speed)
		d.engine.rpm += (math.max(target, d.engine.idle_rpm) - d.engine.rpm) * d.converter_coupling * 4.0 * dt
		d.engine.rpm = math.clamp(d.engine.rpm, 0, d.engine.rev_limiter_rpm)
	else
		Drivetrain.updateEngine(d.engine, d.throttle_input, dt)
	end

	-- Idle / stall
	if d.throttle_input < 0.05 and d.transmission.clutch_engagement > 0.5 then
		if d.engine.rpm > d.engine.idle_rpm + 100 then
			d.engine.rpm -= d.engine.engine_braking * dt
		end
	end
	d.is_stalled = d.transmission.clutch_engagement > 0.5
		and d.engine.rpm < d.engine.idle_rpm * 0.7
		and math.abs(d.wheel_speed) < 0.5
	if d.is_stalled then d.engine.rpm = d.engine.idle_rpm end

	-- Rev limiter
	d.engine.rpm = math.min(d.engine.rpm, d.engine.rev_limiter_rpm)
end

function Drivetrain.requestShiftUp(d)
	if d.shift.phase ~= "Idle" then return false end
	if d.transmission.current_gear >= #d.transmission.gear_ratios then return false end
	d.shift.pending_gear = d.transmission.current_gear + 1
	d.shift.phase = "Disengaging"
	d.shift.timer = d.shift_duration * d.shift.duration_multiplier / 3.0
	return true
end

function Drivetrain.requestShiftDown(d)
	if d.shift.phase ~= "Idle" then return false end
	if d.transmission.current_gear <= -1 then return false end
	d.shift.pending_gear = d.transmission.current_gear - 1
	if d.shift.pending_gear < 0 and math.abs(d.wheel_speed) > 0.5 then
		d.shift.pending_gear = d.transmission.current_gear
		return false
	end
	d.shift.phase = "Disengaging"
	d.shift.timer = d.shift_duration * d.shift.duration_multiplier / 3.0
	return true
end

function Drivetrain.applyAutomaticShifting(d)
	if d.transmission.mode ~= "Automatic" then return end
	if d.shift.phase ~= "Idle" then return end
	if Drivetrain.autoShouldUpshift(d.auto_shift, d.transmission.current_gear, d.engine.rpm, d.throttle_input, d.time) then
		Drivetrain.requestShiftUp(d)
	elseif Drivetrain.autoShouldDownshift(d.auto_shift, d.transmission.current_gear, d.engine.rpm, d.time) then
		Drivetrain.requestShiftDown(d)
	end
end

-- ============================================================
-- TCM (Transmission Control Module) — simplified for now
-- ============================================================

function Drivetrain.newTCM(overrides)
	local t = {
		enabled = true,
		line_pressure = 1.0,
		converter_lockup = false,
		lockup_speed_threshold = 40.0,
		lockup_throttle_threshold = 0.3,
		thermal_limit = 120.0,
		limp_mode = false,
		limp_gear = 3,
		shift_latency = 0.0,
		torque_reduction_request = 0.0,
		fault_clock = 0.0,
		fault_mask = 0,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

-- ============================================================
-- Drive modes
-- ============================================================

local DRIVE_MODE_PROFILES = {
	Normal  = { throttle_map = 1.0, throttle_response = 1.0, idle_rpm_offset = 0, engine_braking_factor = 1.0, steering_assist = 1.0, steering_response = 1.0, suspension_damping = 1.0, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0 },
	Eco     = { throttle_map = 0.7, throttle_response = 0.8, idle_rpm_offset = -100, engine_braking_factor = 1.2, steering_assist = 1.2, steering_response = 0.9, suspension_damping = 0.9, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0 },
	Comfort = { throttle_map = 0.85, throttle_response = 0.9, idle_rpm_offset = 0, engine_braking_factor = 0.8, steering_assist = 1.3, steering_response = 0.85, suspension_damping = 0.8, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0 },
	Sport   = { throttle_map = 1.1, throttle_response = 1.2, idle_rpm_offset = 200, engine_braking_factor = 1.3, steering_assist = 0.8, steering_response = 1.2, suspension_damping = 1.2, diff_mode = "LimitedSlip", abs_mod = 1.2, tc_mod = 1.3, vsc_mod = 1.2 },
	Track   = { throttle_map = 1.2, throttle_response = 1.3, idle_rpm_offset = 300, engine_braking_factor = 1.5, steering_assist = 0.6, steering_response = 1.3, suspension_damping = 1.5, diff_mode = "Locked", abs_mod = 1.5, tc_mod = 1.5, vsc_mod = 1.5 },
	Snow    = { throttle_map = 0.6, throttle_response = 0.7, idle_rpm_offset = 0, engine_braking_factor = 0.7, steering_assist = 1.3, steering_response = 0.7, suspension_damping = 0.7, diff_mode = "LimitedSlip", abs_mod = 0.8, tc_mod = 0.7, vsc_mod = 0.7 },
}

function Drivetrain.getDriveModeProfile(mode)
	return DRIVE_MODE_PROFILES[mode] or DRIVE_MODE_PROFILES.Normal
end

function Drivetrain.mapThrottle(profile, raw_throttle)
	return math.clamp(raw_throttle * profile.throttle_map, 0, 1)
end

return Drivetrain
