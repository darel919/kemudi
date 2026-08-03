--[[
	Safety systems — ABS, traction control, VSC/ESC, ADAS.
	Ported from crates/kemudi-engine/src/safety.rs.
	Pure Luau, no Roblox API dependencies.
--]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")

local Safety = {}

-- ============================================================
-- ABS
-- ============================================================

function Safety.newABS(overrides)
	local a = {
		enabled = true,
		mode = "Enabled", -- Enabled | Sport | Disabled | Degraded | Faulted
		wheel_pressure = { 1.0, 1.0, 1.0, 1.0 },
		slip_threshold = 0.15,
		valve_latency = 0.01,
		pump_latency = 0.02,
		is_active = false,
		sensor_ok = { true, true, true, true },
	}
	if overrides then for k, v in overrides do a[k] = v end end
	return a
end

-- Returns per-wheel brake pressure multipliers [4].
function Safety.updateABS(abs, wheel_speeds, vehicle_speed, brake_command, dt)
	if not abs.enabled or abs.mode == "Disabled" or abs.mode == "Faulted" then
		return { brake_command, brake_command, brake_command, brake_command }
	end

	local threshold = if abs.mode == "Sport" then abs.slip_threshold * 1.5 else abs.slip_threshold
	local any_active = false

	for i = 1, 4 do
		if not abs.sensor_ok[i] then
			abs.wheel_pressure[i] = brake_command * 0.6
			continue
		end

		if vehicle_speed > 0.5 then
			local slip = 1.0 - (wheel_speeds[i] / vehicle_speed)
			slip = math.clamp(slip, 0, 1.5)

			if slip > threshold and brake_command > 0.1 then
				local reduction = math.clamp((slip - threshold) / threshold, 0, 1)
				abs.wheel_pressure[i] = brake_command * (1 - reduction * 0.7)
				any_active = true
			else
				abs.wheel_pressure[i] = math.min(abs.wheel_pressure[i] + 2.0 * dt, brake_command)
			end
		else
			abs.wheel_pressure[i] = brake_command
		end
	end

	abs.is_active = any_active
	return { abs.wheel_pressure[1], abs.wheel_pressure[2], abs.wheel_pressure[3], abs.wheel_pressure[4] }
end

-- ============================================================
-- Traction Control
-- ============================================================

function Safety.newTractionControl(overrides)
	local t = {
		enabled = false,
		mode = "Enabled",
		slip_threshold = 0.12,
		throttle_reduction_max = 0.5,
		brake_intervention = false,
		throttle_modifier = 1.0,
		is_active = false,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

-- Returns throttle modifier (0–1).
function Safety.updateTractionControl(tc, wheel_speeds, vehicle_speed, driven_wheels, throttle_command, dt)
	if not tc.enabled or tc.mode == "Disabled" or tc.mode == "Faulted" then
		return 1.0
	end
	if vehicle_speed <= 0.5 then return 1.0 end

	local threshold = if tc.mode == "Sport" then tc.slip_threshold * 2.0 else tc.slip_threshold
	tc.throttle_modifier = 1.0
	tc.is_active = false

	for _, idx in driven_wheels do
		local i = idx + 1 -- 0-indexed input → 1-indexed Luau
		if i < 1 or i > 4 then continue end
		local slip = math.max(wheel_speeds[i] / vehicle_speed - 1.0, 0)
		if slip > threshold and throttle_command > 0.1 then
			local excess = (slip - threshold) / threshold
			local reduction = math.min(excess * tc.throttle_reduction_max, tc.throttle_reduction_max)
			tc.throttle_modifier = math.max(1.0 - reduction, 0.3)
			tc.is_active = true
		end
	end

	return tc.throttle_modifier
end

-- ============================================================
-- VSC / ESC (Yaw Stability Control)
-- ============================================================

function Safety.newVSC(overrides)
	local v = {
		enabled = false,
		mode = "Enabled",
		yaw_rate_sensor = true,
		lateral_accel_sensor = true,
		individual_brake = true,
		torque_reduction = true,
		yaw_error_threshold = 0.12,
		is_active = false,
		brake_torque = { 0, 0, 0, 0 },
		torque_modifier = 1.0,
	}
	if overrides then for k, v2 in overrides do v[k] = v2 end end
	return v
end

-- Returns: per-wheel brake torque [4], engine torque modifier.
function Safety.updateVSC(vsc, actual_yaw_rate, desired_yaw_rate, steering_angle, vehicle_speed, dt)
	if not vsc.enabled or vsc.mode == "Disabled" or vsc.mode == "Faulted" then
		return { 0, 0, 0, 0 }, 1.0
	end
	if vehicle_speed <= 5.0 then
		return { 0, 0, 0, 0 }, 1.0
	end

	local threshold = if vsc.mode == "Sport" then vsc.yaw_error_threshold * 1.5 else vsc.yaw_error_threshold
	local yaw_error = actual_yaw_rate - desired_yaw_rate

	-- Turn direction
	local turn_sign = Math.sign(steering_angle)
	if turn_sign == 0 then turn_sign = Math.sign(desired_yaw_rate) end
	if turn_sign == 0 then turn_sign = 1 end

	local signed_error = yaw_error * turn_sign
	local brakes = { 0, 0, 0, 0 }
	vsc.is_active = false

	if signed_error > threshold then
		-- Oversteer
		vsc.is_active = true
		if turn_sign > 0 then
			brakes[1] = math.abs(yaw_error) * 15.0 -- front-left
		else
			brakes[2] = math.abs(yaw_error) * 15.0 -- front-right
		end
	elseif signed_error < -threshold then
		-- Understeer
		vsc.is_active = true
		if turn_sign > 0 then
			brakes[4] = math.abs(yaw_error) * 10.0 -- rear-right
		else
			brakes[3] = math.abs(yaw_error) * 10.0 -- rear-left
		end
	end

	-- Engine torque reduction
	local reduction = math.min(math.abs(yaw_error) / math.max(threshold, 1e-6) * 0.12, 0.25)
	vsc.torque_modifier = 1.0 - reduction
	vsc.brake_torque = brakes

	return brakes, vsc.torque_modifier
end

-- ============================================================
-- ADAS
-- ============================================================

function Safety.newADAS(overrides)
	local a = {
		forward_collision_warning = false,
		automatic_emergency_braking = false,
		adaptive_cruise_control = false,
		blind_spot_warning = false,
		lane_departure_warning = false,
		lane_departure_assist = false,
		parking_sensors = false,
		perception_range = 200.0,
		radar_fov = 20.0,
		camera_fov = 50.0,
		sensor_faults = "None",
		front_target = { detected = false, distance = 0, relative_speed = 0, confidence = 0, time_to_collision = 0 },
		blind_spot = {
			{ detected = false, distance = 0, relative_speed = 0, confidence = 0, time_to_collision = 0 },
			{ detected = false, distance = 0, relative_speed = 0, confidence = 0, time_to_collision = 0 },
		},
		aeb_active = false,
		acc_target_speed = 0.0,
		ldw_active = false,
		braking_authority = 0.7,
		_forward_target_confirmation = 0,
		_aeb_ttc_threshold = 0.8,
	}
	if overrides then for k, v in overrides do a[k] = v end end
	return a
end

function Safety.updateForwardCollision(adas, target_distance, target_relative_speed, vehicle_speed, dt)
	-- Sensor fault → clear
	if adas.sensor_faults ~= "None" then
		adas.front_target.detected = false
		adas.front_target.confidence = 0
		adas._forward_target_confirmation = 0
		return
	end

	local in_range = target_distance > 0 and target_distance < adas.perception_range
	local closing = vehicle_speed > 0.5 and target_relative_speed < 0

	adas.front_target.distance = target_distance
	adas.front_target.relative_speed = target_relative_speed

	if in_range then
		adas.front_target.detected = true
		adas.front_target.confidence = 0.8
	else
		adas.front_target.detected = false
		adas.front_target.confidence = 0
	end

	-- Debounce
	if in_range and closing then
		adas._forward_target_confirmation = math.min(adas._forward_target_confirmation + 1, 3)
	else
		adas._forward_target_confirmation = 0
	end

	-- Speed-adaptive AEB threshold
	adas._aeb_ttc_threshold = math.clamp(0.8 + 0.04 * vehicle_speed, 0.8, 2.0)

	-- TTC
	if closing then
		adas.front_target.time_to_collision = target_distance / math.max(-target_relative_speed + 0.1, 0.01)
	else
		adas.front_target.time_to_collision = math.huge
	end

	-- FCW
	local fcw_threshold = math.clamp(2.5 + 0.02 * vehicle_speed, 2.5, 3.1)
	adas.forward_collision_warning = adas._forward_target_confirmation >= 2
		and adas.front_target.detected
		and adas.front_target.time_to_collision < fcw_threshold
		and adas.front_target.confidence > 0.3

	-- AEB
	adas.aeb_active = adas.automatic_emergency_braking
		and adas._forward_target_confirmation >= 3
		and adas.front_target.detected
		and adas.front_target.time_to_collision < adas._aeb_ttc_threshold
		and adas.front_target.confidence > 0.5
end

function Safety.getAEBBrake(adas, brake_authority)
	if not adas.aeb_active then return 0 end

	local ttc = adas.front_target.time_to_collision
	local threshold = adas._aeb_ttc_threshold
	local relative_speed = adas.front_target.relative_speed
	local distance = adas.front_target.distance

	-- TTC-based urgency
	local urgency = math.clamp(1 - ttc / math.max(threshold, 0.01), 0, 1)
	local shaped = urgency * urgency * (3 - 2 * urgency) -- smoothstep
	local ttc_demand = 0.15 + 0.85 * shaped

	-- Physics-based demand
	local closing_speed = math.max(-relative_speed, 0)
	local required_decel = closing_speed * closing_speed / math.max(2 * distance, 0.01)
	local required_demand = math.clamp(required_decel / 7.5, 0, 1)

	local demand = math.max(ttc_demand, required_demand)
	return demand * adas.braking_authority * brake_authority
end

function Safety.updateBlindSpot(adas, left_distance, right_distance, vehicle_speed)
	if adas.sensor_faults ~= "None" or not adas.blind_spot_warning then
		adas.blind_spot[1].detected = false
		adas.blind_spot[2].detected = false
		return
	end

	for i, dist in { left_distance, right_distance } do
		if dist > 0 and dist < 10 then
			adas.blind_spot[i].detected = true
			adas.blind_spot[i].distance = dist
			adas.blind_spot[i].confidence = 0.7
			adas.blind_spot[i].time_to_collision = math.huge
		else
			adas.blind_spot[i].detected = false
			adas.blind_spot[i].confidence = 0
		end
	end
end

-- ============================================================
-- Aggregate container
-- ============================================================

function Safety.newState(overrides)
	local s = {
		abs = Safety.newABS(),
		traction_control = Safety.newTractionControl(),
		vsc = Safety.newVSC(),
		adas = Safety.newADAS(),
	}
	if overrides then
		if overrides.abs then for k, v in overrides.abs do s.abs[k] = v end end
		if overrides.traction_control then for k, v in overrides.traction_control do s.traction_control[k] = v end end
		if overrides.vsc then for k, v in overrides.vsc do s.vsc[k] = v end end
		if overrides.adas then for k, v in overrides.adas do s.adas[k] = v end end
	end
	return s
end

return Safety
