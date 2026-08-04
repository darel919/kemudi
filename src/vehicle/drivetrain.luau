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

function Drivetrain.newEngine(overrides)
	assert(type(overrides) == "table", "engine parameters must be authored by the vehicle asset")
	local e = {
		rpm = overrides.idle_rpm,
		idle_rpm = overrides.idle_rpm,
		redline_rpm = overrides.redline_rpm,
		rev_limiter_rpm = overrides.rev_limiter_rpm,
		stall_rpm = overrides.stall_rpm or 450,
		torque_curve = overrides.torque_curve,
		throttle_response = overrides.throttle_response,
		engine_braking = overrides.engine_braking,
		inertia = assert(overrides.inertia, "engine inertia must be authored by the vehicle asset"),
	}
	if overrides then for k, v in overrides do e[k] = v end end
	return e
end

function Drivetrain.torqueAtRPMFromCurve(curve, rpm)
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

function Drivetrain.engineTorqueAtRPM(engine, rpm)
	return Drivetrain.torqueAtRPMFromCurve(engine.torque_curve, rpm)
end

function Drivetrain.updateEngine(engine, throttle, dt, shaft_rpm, coupling, external_load_fraction)
	local torque = Drivetrain.engineTorqueAtRPM(engine, engine.rpm) * math.clamp(throttle or 0, 0, 1)
	-- A slipping automatic converter still loads the impeller. Without this
	-- reaction torque, the engine can free-rev to the limiter in D at standstill
	-- because only the small shaft-synchronization term remains. Park/Neutral
	-- pass zero load from the caller so the engine can free-rev there.
	local converterLoad = math.clamp(external_load_fraction or 0, 0, 1)
	torque -= torque * converterLoad
	local rpmFraction = math.clamp(engine.rpm / math.max(engine.redline_rpm, 1), 0, 1)
	local friction = engine.engine_braking * (0.25 + rpmFraction * 0.75)
	local syncTorque = 0
	if type(shaft_rpm) == "number" and shaft_rpm == shaft_rpm then
		local angularError = (shaft_rpm - engine.rpm) * (2 * math.pi / 60)
		local syncGain = math.max(engine.throttle_response, 0.1) * 2.0
		syncTorque = angularError * math.max(engine.inertia, 0.01) * syncGain * math.clamp(coupling or 0, 0, 1)
	end
	local angularAcceleration = (torque - friction + syncTorque) / math.max(engine.inertia, 0.01)
	engine.rpm += angularAcceleration * (60 / (2 * math.pi)) * math.max(dt, 0)
	-- Idle air control: the engine always maintains at least idle RPM when
	-- running. Previously this was gated on throttle > 0.01, which let RPM
	-- decay to zero whenever the driver lifted off — the torque converter
	-- slipping case now has no syncTorque to arrest the decay.
	if engine.rpm < engine.idle_rpm then
		engine.rpm = engine.idle_rpm
	end
	engine.rpm = math.clamp(engine.rpm, 0, engine.rev_limiter_rpm)
end

-- ============================================================
-- Transmission
-- ============================================================

function Drivetrain.newTransmission(overrides)
	assert(type(overrides) == "table", "transmission parameters must be authored by the vehicle asset")
	local t = {
		mode = overrides.mode,
		gear_ratios = overrides.gear_ratios,
		final_drive = overrides.final_drive,
		current_gear = overrides.current_gear or 1,
		reverse_ratio = overrides.reverse_ratio,
		clutch_engagement = overrides.clutch_engagement,
		selector = if overrides.mode == "Automatic" then "P" else "M/S",
		requested_selector = nil,
		last_rejection = nil,
		last_rejection_code = 0,
		last_warning = nil,
		last_warning_code = 0,
		pending_shift_damage = 0.0,
		pending_shift_shock = 0.0,
		damage = 0.0,
		wear = 0.0,
		abuse_stress = 0.0,
		torque_derate = 1.0,
		failure_state = "Healthy",
		is_failed = false,
		failure_reason = nil,
	}
	if overrides then for k, v in overrides do t[k] = v end end
	if t.mode == "Automatic" then
		-- Every automatic spawn starts mechanically in Park, regardless of a
		-- stale serialized gear value. The selector must be moved through the
		-- interlocked sequence before a drive ratio can engage.
		t.selector = "P"
		t.current_gear = 0
	elseif t.selector == "P" or t.selector == "N" or t.selector == "R" then
		t.selector = "M/S"
		t.current_gear = math.max(t.current_gear, 1)
	end
	return t
end

function Drivetrain.totalRatio(trans)
	if trans.selector == "P" or trans.selector == "N" then return 0 end
	local gear = trans.current_gear
	if gear >= 1 and gear <= #trans.gear_ratios then
		return trans.gear_ratios[gear] * trans.final_drive
	elseif gear == -1 then
		return -trans.reverse_ratio * trans.final_drive
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
	assert(type(overrides) == "table" and type(overrides.engine) == "table" and type(overrides.transmission) == "table", "vehicle drivetrain must include authored engine and transmission parameters")
	local d = {
		engine = Drivetrain.newEngine(overrides.engine),
		transmission = Drivetrain.newTransmission(overrides.transmission),
		differential = Drivetrain.newDifferential(),
		wheel_speed = 0.0,
		throttle_input = 0.0,
		brake_input = 0.0,
		clutch_input = 0.0,
		shift = Drivetrain.newShiftState(),
		shift_duration = 0.15,
		auto_shift = Drivetrain.newAutoShift(),
		time = 0.0,
		engine_stage = 0,
		is_stalled = false,
		last_drive_torque = 0.0,
		converter_coupling = 1.0,
		converter_torque_multiplier = 2.0,
		transmission_temperature = 20.0,
		wheel_radius = 0.3,
		wheel_inertia = 0.0,
		max_downshift_rpm = 8000.0,
		clutch_shock = Drivetrain.newClutchShock(),
		tcm = Drivetrain.newTCM(),
		overrev_events = {},
	}
	if overrides then
		for k, v in overrides do
			if k ~= "engine" and k ~= "transmission" and k ~= "differential" and k ~= "tcm" then
				d[k] = v
			end
		end
		if overrides.engine then for k, v in overrides.engine do d.engine[k] = v end end
		if overrides.transmission then for k, v in overrides.transmission do d.transmission[k] = v end end
		if overrides.differential then for k, v in overrides.differential do d.differential[k] = v end end
		if overrides.tcm then d.tcm = Drivetrain.newTCM(overrides.tcm) end
	end
	d.tcm.gear_ratios = d.transmission.gear_ratios
	d.tcm.final_drive = d.transmission.final_drive
	d.tcm.engine_idle_rpm = d.engine.idle_rpm
	d.tcm.engine_rev_limiter_rpm = d.engine.rev_limiter_rpm
	d.tcm.torque_curve = d.engine.torque_curve
	local peakTorque = 0
	for _, point in d.engine.torque_curve do peakTorque = math.max(peakTorque, point[2]) end
	d.tcm.peak_torque = peakTorque
	d.drive_mode = overrides.drive_mode or d.tcm.drive_mode or "Normal"
	d.tcm.drive_mode = d.drive_mode
	return d
end

function Drivetrain.effectiveWheelInertia(dt, vehicle_mass, wheel_inertia)
	if wheel_inertia > 0 then return wheel_inertia end
	return math.max(vehicle_mass * 0.01, 0.1)
end

-- Find peak-torque RPM from the authored torque curve. Returns (peakRPM, peakTorque).
local function peakTorqueRPM(engine)
	if not engine or not engine.torque_curve then return 0, 0 end
	local bestRPM, bestT = 0, 0
	for _, pt in engine.torque_curve do
		if pt[2] > bestT then bestT = pt[2]; bestRPM = pt[1] end
	end
	return bestRPM, bestT
end

function Drivetrain.updateTransmissionHealth(d, dt)
	local transmission = d.transmission
	if transmission.is_failed then return end
	local safeDt = math.clamp(dt or 0, 0, 0.25)
	if safeDt <= 0 then return end

	-- --- Thermal ---
	local thermalLimit = if d.tcm and d.tcm.thermal_limit < math.huge
		then math.max(d.tcm.thermal_limit, 80)
		else 120
	local thermalSeverity = math.max((d.transmission_temperature - thermalLimit) / 30, 0)

	-- --- Clutch / shock ---
	local shockTorque = math.abs(d.clutch_shock and d.clutch_shock.torque_spike or 0)
	shockTorque = math.max(shockTorque, transmission.pending_shift_shock or 0)
	local shockSeverity = math.clamp((shockTorque - 800) / 2400, 0, 6)

	-- --- Torque overload ---
	local ratio = math.abs(Drivetrain.totalRatio(transmission))
	local inputTorque = if ratio > 1e-6 then math.abs(d.last_drive_torque) / ratio else 0
	local peakTorque = if d.tcm and d.tcm.peak_torque > 0 then d.tcm.peak_torque else 500
	local torqueSeverity = math.max(inputTorque / peakTorque - 1.25, 0)

	-- --- Overrev (shaft RPM past limiter) ---
	local shaftRpm = Drivetrain.engineRPMFromWheelSpeed(transmission, d.wheel_speed)
	local limiter = if d.engine and d.engine.rev_limiter_rpm > 0 then d.engine.rev_limiter_rpm else 7000
	local overrevSeverity = math.max((shaftRpm - limiter) / limiter, 0)

	-- --- Shift RPM mismatch ---
	-- Large RPM delta between engine and coupled shaft at the moment of
	-- gear engagement stresses synchros and clutch packs. Consumed once.
	local shiftMismatch = transmission._shift_rpm_mismatch or 0
	transmission._shift_rpm_mismatch = nil
	-- 500 RPM is a normal well-timed shift; above that is increasingly harsh.
	local mismatchSeverity = math.clamp((shiftMismatch - 500) / 1000, 0, 4)
	if mismatchSeverity > 0 then
		transmission.pending_shift_damage = math.max(
			transmission.pending_shift_damage or 0,
			mismatchSeverity * 0.008)
	end

	-- --- Power band violation (continuous) ---
	-- The transmission's job is to keep the engine in its efficient RPM range.
	-- Operating outside that range under load means the TCM chose the wrong gear.
	-- Guard: below 15 km/h the engine is expected to be at low RPM (idle/creep).
	local vehicleSpeedKmh = d._vehicle_speed_kmh or 0
	local bandSeverity = 0
	if vehicleSpeedKmh > 15 then
		local engineRpm = if d.engine then d.engine.rpm else 0
		local peakRPM = peakTorqueRPM(d.engine)
		local bandLow = if peakRPM > 0 then peakRPM else limiter * 0.45
		local bandHigh = limiter * 0.80
		local load = if d.tcm then d.tcm.engine_load or 0 else 0
		-- Lugging: RPM well below power band under heavy load
		if load > 0.6 and engineRpm < bandLow and bandLow > 0 then
			bandSeverity = math.clamp((bandLow - engineRpm) / bandLow, 0, 1) * load * 0.6
		end
		-- Over-rev approach: RPM near redline under load (should have upshifted)
		if load > 0.3 and engineRpm > bandHigh and bandHigh > 0 then
			bandSeverity += math.clamp((engineRpm - bandHigh) / (limiter - bandHigh), 0, 1) * load * 0.4
		end
	end

	-- --- Combine ---
	local abuseSeverity = math.max(thermalSeverity, shockSeverity, torqueSeverity, overrevSeverity, mismatchSeverity, bandSeverity)

	local pendingDamage = math.clamp(transmission.pending_shift_damage or 0, 0, 1)
	transmission.pending_shift_damage = 0
	transmission.pending_shift_shock = 0
	if abuseSeverity > 0 or pendingDamage > 0 then
		transmission.abuse_stress = math.clamp(transmission.abuse_stress + abuseSeverity * 0.7 * safeDt, 0, 2)
		transmission.damage = math.clamp(transmission.damage + (
			thermalSeverity * 0.006
			+ shockSeverity * 0.02
			+ torqueSeverity * 0.004
			+ overrevSeverity * 0.01
			+ mismatchSeverity * 0.008
			+ bandSeverity * 0.003
		) * safeDt + pendingDamage, 0, 1)
	else
		transmission.abuse_stress = math.max(transmission.abuse_stress - 0.15 * safeDt, 0)
	end
	transmission.wear = math.max(transmission.wear, transmission.damage)

	if transmission.damage >= 1 then
		transmission.damage = 1
		transmission.wear = 1
		transmission.torque_derate = 0
		transmission.failure_state = "Failed"
		transmission.is_failed = true
		transmission.failure_reason = "excessive_mechanical_stress"
		transmission.selector = "N"
		transmission.current_gear = 0
		transmission.requested_selector = nil
		d.shift.pending_gear = nil
		d.shift.phase = "Idle"
		if d.tcm then
			d.tcm.limp_mode = true
			d.tcm.state = "Fault"
			d.tcm.torque_reduction_request = 1
			d.tcm.converter_lockup = false
			d.tcm.last_diagnostic_code = 722
		end
	elseif transmission.damage >= 0.6 then
		transmission.failure_state = "Severe"
		transmission.torque_derate = math.clamp(1 - transmission.damage * 0.75, 0.2, 1)
	elseif transmission.damage >= 0.2 then
		transmission.failure_state = "Damaged"
		transmission.torque_derate = math.clamp(1 - transmission.damage * 0.75, 0.35, 1)
	else
		transmission.failure_state = "Healthy"
		transmission.torque_derate = 1
	end
end

function Drivetrain.update(dt, vehicle_mass, d)
	d.time += dt
	d.throttle_input = math.clamp(d.throttle_input, 0, 1)
	d.brake_input = math.clamp(d.brake_input, 0, 1)
	d.clutch_input = math.clamp(d.clutch_input, 0, 1)
	-- Engine stall detection: RPM below stall threshold while engine is
	-- running (stage 2) → engine dies. The runtime detects is_stalled
	-- after the physics step and transitions to OFF.
	if d.engine_stage == 2 and d.engine.rpm < d.engine.stall_rpm and not d.is_stalled then
		d.is_stalled = true
	end

	-- Shift phase update
	local shift = d.shift
	if shift.phase == "Disengaging" then
		shift.timer -= dt
		if shift.timer <= 0 then
			-- Capture RPM mismatch at the moment of gear change.
			-- The clutch/converter must absorb this delta on re-engagement;
			-- large mismatches stress synchros and clutch packs.
			local preShiftRPM = d.engine and d.engine.rpm or 0
			d.transmission.current_gear = shift.pending_gear
			local postShiftCoupled = Drivetrain.engineRPMFromWheelSpeed(d.transmission, d.wheel_speed)
			d.transmission._shift_rpm_mismatch = math.abs(preShiftRPM - postShiftCoupled)
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
			if d.tcm then d.tcm.pending_gear = nil end
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

	-- ISC (Idle Speed Control): maintain idle RPM when converter loads engine.
	-- Two components: base idle throttle (minimum in gear for creep) and
	-- RPM correction (proportional to deficit below idle target). Without
	-- this, zero throttle → zero engine torque → no creep in D/R.
	-- ponytail: base idle + linear correction; upgrade to PID when
	-- authored ISC maps are available.
	local isParked = d.transmission.selector == "P" or d.transmission.selector == "N"
	local effective_throttle = d.throttle_input
	if d.throttle_input < 0.05
		and d.transmission.mode == "Automatic"
		and d.converter_coupling > 0.01
		and not isParked then
		local base_idle = 0.05
		local rpm_deficit = math.max(d.engine.idle_rpm - d.engine.rpm, 0)
		local rpm_correction = math.clamp(rpm_deficit / math.max(d.engine.idle_rpm, 1) * 0.20, 0, 0.10)
		effective_throttle = math.max(effective_throttle, math.clamp(base_idle + rpm_correction, 0, 0.15))
	end

	-- Engine torque
	local engine_torque
	if rigid_coupling and coupled_rpm >= d.engine.rev_limiter_rpm then
		engine_torque = 0
	else
		local tcm_reduction = if d.tcm then math.clamp(d.tcm.torque_reduction_request, 0, 0.8) else 0
		engine_torque = Drivetrain.engineTorqueAtRPM(d.engine, d.engine.rpm) * effective_throttle * (1 - tcm_reduction)
	end
	-- Rev limiter fuel/spark cut: zero torque when RPM approaches the
	-- limiter. Without this, the engine produces full torque at the
	-- limiter during kickdown — RPM is clamped but torque persists,
	-- causing thermal damage and overrev explosion.
	-- ponytail: simple threshold with 200 RPM hysteresis; upgrade to
	-- per-cylinder fuel-cut model for realistic limiter bounce sound.
	local limiter_cut = d.engine.rev_limiter_rpm - 200
	if d.engine.rpm >= limiter_cut then
		engine_torque = 0
	end

	-- Torque multiplication: stator multiplies torque at high slip.
	-- Efficiency rises with coupling — at near-stall most torque is
	-- lost to fluid friction. Floor of 0.15 models minimum fluid
	-- coupling that transmits torque for creep.
	local slip = math.clamp(1 - d.converter_coupling, 0, 1)
	local torqueRatio = 1 + (d.converter_torque_multiplier - 1) * math.clamp(slip / 0.85, 0, 1)
	local converter_factor = if d.transmission.mode == "Automatic"
		then math.clamp(torqueRatio * math.max(d.converter_coupling, 0.15), 0, 2.5)
		else 1.0
	local drive_torque = Drivetrain.wheelTorque(d.transmission, engine_torque)
		* converter_factor
		* math.clamp(d.transmission.torque_derate or 1, 0, 1)

	-- Angular integration
	local wheel_inertia = Drivetrain.effectiveWheelInertia(dt, vehicle_mass, d.wheel_inertia)
	local stopping_torque = math.abs(d.wheel_speed) * wheel_inertia / math.max(dt, 1e-6)

	-- Engine braking
	local requested_engine_brake = 0
	if effective_throttle < 0.05 and math.abs(d.wheel_speed) > 1e-6 and d.transmission.clutch_engagement > 0.05 then
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
	-- Transmission thermal model.
	-- Heat input: converter slip loss + gear/bearing losses.
	-- Cooling: radiator-integrated transmission oil cooler (TOC) — a
	-- liquid-to-liquid heat exchanger that uses engine coolant as the
	-- thermal reference, plus airflow-dependent convective cooling.
	-- The TOC is the primary cooling path in real cars; without it the
	-- transmission cannot reject heat fast enough under sustained load.
	local outputPower = math.abs(shaft_torque * d.wheel_speed)
	local coupling = math.clamp(d.converter_coupling or 0, 0, 1)
	local lossFactor = if d.transmission.mode == "Automatic"
		then 0.02 + (1 - coupling)
		else 0.02
	local heat = outputPower * lossFactor * 0.00008

	-- TOC reference: engine coolant temperature (set by vehicle.luau from
	-- the previous frame's engine telemetry). Falls back to ambient when
	-- no thermal model is present.
	local coolantRef = d._toc_coolant_temp or 20
	local transDelta = d.transmission_temperature - coolantRef
	local vehicleSpeedKmh = d._vehicle_speed_kmh or 0

	-- TOC heat-exchanger rate: bidirectional, proportional to temperature
	-- difference between ATF and coolant. Heats cold ATF toward coolant
	-- temp; cools hot ATF when above coolant. Settles ATF ~5-10 C above
	-- coolant during normal driving; higher under heavy load.
	-- Rate is high because the TOC is a liquid-to-liquid exchanger
	-- with large surface area inside the radiator core.
	local tocRate = 0.2
	-- Airflow cooling: auxiliary cooler + transmission pan ventilation,
	-- increases with vehicle speed. Full effect above ~80 km/h.
	local airflowRate = 0.03 * math.clamp(vehicleSpeedKmh / 80, 0, 1)
	local cooling = transDelta * tocRate + math.max(transDelta, 0) * airflowRate
	d.transmission_temperature = math.clamp(d.transmission_temperature + (heat - cooling) * dt, 20, 200)

	-- RPM coupling: engine speed is torque/inertia driven and is pulled toward
	-- the input-shaft speed by clutch/converter coupling. Throttle no longer
	-- teleports RPM directly to a redline target.
	--
	-- The converter provides partial coupling at all speeds — even when
	-- "slipping" the fluid transmits enough torque to prevent the engine
	-- from free-revving to redline. Using the actual coupling value
	-- (not a binary 0/1 threshold) lets the engine RPM drop naturally
	-- after an upshift and prevents the RPM staying pinned at limiter.
	--
	-- In P/N there is no load path to the wheels, so coupling is zero
	-- and the engine can rev freely.
	local inParkNeutral = d.transmission.selector == "P" or d.transmission.selector == "N"
	local shaftTarget = if shift.phase == "Neutral" or inParkNeutral then nil else coupled_rpm
	local shaftCoupling = if shift.phase == "Neutral" or inParkNeutral then 0
		elseif d.transmission.mode == "Automatic" then d.converter_coupling
		else d.transmission.clutch_engagement
	-- Converter parasitic load scales with engine speed: at high RPM the
	-- fluid couples strongly and absorbs enough torque to prevent free-
	-- revving; at low RPM (launch) the load is small so the engine can
	-- produce wheel torque. A flat cap either lets the engine hit the
	-- limiter in D or starves it at launch.
	local slipLoad = math.clamp(1 - d.converter_coupling, 0, 1)
	local rpmFraction = math.clamp(d.engine.rpm / math.max(d.engine.redline_rpm, 1), 0, 1)
	local converterLoad = if d.transmission.mode == "Automatic"
		and not inParkNeutral
		and shift.phase ~= "Neutral"
		then math.clamp(slipLoad * rpmFraction, 0, 0.95)
		else 0
	Drivetrain.updateEngine(d.engine, effective_throttle, dt, shaftTarget, shaftCoupling, converterLoad)

	-- Idle / stall
	if d.throttle_input < 0.05 and d.transmission.clutch_engagement > 0.5 then
		if d.engine.rpm > d.engine.idle_rpm + 100 then
			d.engine.rpm -= d.engine.engine_braking * dt
		end
	end
	-- Engine stall: RPM below stall threshold → engine dies.
	-- The stall detection at the top of update() sets is_stalled=true;
	-- here we enforce zero RPM so the engine doesn't keep spinning.
	if d.is_stalled then
		d.engine.rpm = 0
	end

	-- Rev limiter
	d.engine.rpm = math.min(d.engine.rpm, d.engine.rev_limiter_rpm)
	Drivetrain.updateTransmissionHealth(d, dt)
end

local SELECTOR_ORDER = { "P", "R", "N", "D", "M/S" }

local function selectorIndex(selector)
	for index, name in SELECTOR_ORDER do
		if name == selector then return index end
	end
	return nil
end

local function registerNeutralEngagementAbuse(d, targetSelector, speedKmh)
	local transmission = d.transmission
	local speedFactor = math.clamp(math.max(speedKmh, 0) / 60, 0, 4)
	local throttle = math.clamp(d.throttle_input or 0, 0, 1)
	local directionShock = if targetSelector == "R" then 1.5 else 1
	local peakTorque = if d.tcm and d.tcm.peak_torque > 0 then d.tcm.peak_torque else 500
	-- Neutral engagement at speed is allowed, but creates a discrete
	-- mechanical event. Throttle and reverse-direction selection amplify it.
	transmission.pending_shift_damage = math.max(
		transmission.pending_shift_damage or 0,
		0.01 * speedFactor * directionShock * (1 + throttle * 2))
	transmission.pending_shift_shock = math.max(
		transmission.pending_shift_shock or 0,
		peakTorque * (0.8 + speedFactor) * directionShock * (1 + throttle * 2))
	transmission.last_warning = "neutral_engagement_damage"
	transmission.last_warning_code = 413
end

local function rejectTransmissionAction(d, reason, code)
	d.transmission.last_rejection = reason
	d.transmission.last_rejection_code = code or 1
	if d.tcm then d.tcm.last_diagnostic_code = code or 1 end
	return false
end

function Drivetrain.requestGear(d, targetGear)
	if d.transmission.is_failed then return rejectTransmissionAction(d, "transmission_failed", 722) end
	local maxGear = #d.transmission.gear_ratios
	if type(targetGear) ~= "number" or targetGear ~= math.floor(targetGear)
		or targetGear < -1 or targetGear > maxGear then
		return rejectTransmissionAction(d, "invalid_gear", 401)
	end
	if d.shift.phase ~= "Idle" then return rejectTransmissionAction(d, "shift_in_progress", 402) end
	if targetGear == d.transmission.current_gear then return true end
	d.shift.pending_gear = targetGear
	d.shift.phase = "Disengaging"
	d.shift.timer = d.shift_duration * d.shift.duration_multiplier / 3.0
	d.transmission.last_rejection = nil
	d.transmission.last_rejection_code = 0
	return true
end

function Drivetrain.requestShiftUp(d)
	return Drivetrain.requestGear(d, d.transmission.current_gear + 1)
end

function Drivetrain.requestShiftDown(d)
	local targetGear = d.transmission.current_gear - 1
	if targetGear < 0 and math.abs(d.wheel_speed) > 0.5 then
		return rejectTransmissionAction(d, "vehicle_moving", 403)
	end
	return Drivetrain.requestGear(d, targetGear)
end

function Drivetrain.requestSelector(d, targetSelector, speedKmh, brake, _engineOn)
	if d.transmission.is_failed then return rejectTransmissionAction(d, "transmission_failed", 722) end
	if d.transmission.mode ~= "Automatic" then
		return rejectTransmissionAction(d, "selector_not_automatic", 404)
	end
	-- A selector detent is a one-shot mechanical intent. Ignore another
	-- detent while the prior hydraulic shift is still completing; it is not a
	-- driver fault and must not produce an alert/chime during normal operation.
	if d.shift.phase ~= "Idle" then return false end
	if selectorIndex(targetSelector) == nil then
		return rejectTransmissionAction(d, "invalid_selector", 405)
	end
	local currentSelector = d.transmission.selector
	local currentIndex = selectorIndex(currentSelector)
	local targetIndex = selectorIndex(targetSelector)
	if math.abs(targetIndex - currentIndex) ~= 1 then
		return rejectTransmissionAction(d, "selector_sequence", 406)
	end
	local speed = math.max(speedKmh or 0, 0)
	local brakeApplied = (brake or 0) > 0.1
	if targetSelector == "P" and speed > 0.5 then
		return rejectTransmissionAction(d, "vehicle_moving", 403)
	end
	if targetSelector == "R" and speed > 0.5 and currentSelector ~= "N" then
		return rejectTransmissionAction(d, "vehicle_moving", 403)
	end
	if (currentSelector == "P" or targetSelector == "P")
		and not brakeApplied then
		return rejectTransmissionAction(d, "brake_interlock", 407)
	end

	local targetGear = d.transmission.current_gear
	if targetSelector == "P" or targetSelector == "N" then
		targetGear = 0
	elseif targetSelector == "R" then
		targetGear = -1
	elseif targetGear <= 0 then
		targetGear = 1
	end
	if (targetSelector == "D" or targetSelector == "M/S") and targetGear == 1 and speed > 0.5 then
		local wheelSpeed = if d.wheel_radius > 0 then (speed / 3.6) / d.wheel_radius else 0
		local projectedRPM = math.abs(wheelSpeed * d.transmission.gear_ratios[1] * d.transmission.final_drive) * 60 / (2 * math.pi)
		local limiter = d.engine and d.engine.rev_limiter_rpm or math.huge
		local neutralEngagement = currentSelector == "N" and (targetSelector == "D" or targetSelector == "R") and speed > 0.5
		if projectedRPM >= limiter * 0.95 and not neutralEngagement then
			return rejectTransmissionAction(d, "projected_overrev", 408)
		end
	end
	if not Drivetrain.requestGear(d, targetGear) then return false end
	if currentSelector == "N" and (targetSelector == "D" or targetSelector == "R") and speed > 0.5 then
		registerNeutralEngagementAbuse(d, targetSelector, speed)
	end
	d.transmission.selector = targetSelector
	d.transmission.requested_selector = nil
	return true
end

function Drivetrain.requestSelectorUp(d, speedKmh, brake, engineOn)
	local index = selectorIndex(d.transmission.selector) or 1
	return Drivetrain.requestSelector(d, SELECTOR_ORDER[math.min(index + 1, #SELECTOR_ORDER)], speedKmh, brake, engineOn)
end

function Drivetrain.requestSelectorDown(d, speedKmh, brake, engineOn)
	local index = selectorIndex(d.transmission.selector) or 1
	return Drivetrain.requestSelector(d, SELECTOR_ORDER[math.max(index - 1, 1)], speedKmh, brake, engineOn)
end

function Drivetrain.authorizeManualShift(tcm, currentGear, targetGear, vehicleSpeedKmh, maxGear)
	if not tcm or not tcm.enabled then return false, "tcm_disabled" end
	if targetGear < 1 or targetGear > maxGear then return false, "gear_limit" end
	if targetGear < currentGear then
		local ratio = tcm.gear_ratios and tcm.gear_ratios[targetGear]
		local wheelSpeed = if tcm.wheel_radius > 0 then (math.max(vehicleSpeedKmh or 0, 0) / 3.6) / tcm.wheel_radius else 0
		local projectedRPM = if ratio
			then math.abs(wheelSpeed * ratio * tcm.final_drive) * 60 / (2 * math.pi)
			else 0
		if projectedRPM >= tcm.engine_rev_limiter_rpm * 0.95 then return false, "projected_overrev" end
	end
	return true, nil
end

function Drivetrain.requestManualShift(d, direction, vehicleSpeedKmh, brake)
	if d.transmission.is_failed then return rejectTransmissionAction(d, "transmission_failed", 722) end
	if d.transmission.selector ~= "M/S" then
		return rejectTransmissionAction(d, "manual_selector_required", 409)
	end
	if direction ~= 1 and direction ~= -1 then
		return rejectTransmissionAction(d, "invalid_shift_command", 410)
	end
	local currentGear = d.transmission.current_gear
	local targetGear = currentGear + direction
	if currentGear <= 0 and direction > 0 then targetGear = 1 end
	local ok, reason = Drivetrain.authorizeManualShift(
		d.tcm, currentGear, targetGear, vehicleSpeedKmh, #d.transmission.gear_ratios)
	if not ok then
		return rejectTransmissionAction(d, reason, if reason == "projected_overrev" then 408 else 411)
	end
	if brake and brake > 0.1 and math.abs(vehicleSpeedKmh or 0) > 0.5 and targetGear < 1 then
		return rejectTransmissionAction(d, "vehicle_moving", 403)
	end
	return Drivetrain.requestGear(d, targetGear)
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
		enabled = false,
		drive_mode = "Normal",
		state = "Normal", -- Normal | Sport | Winter | Manual | Limp | Fault | Off
		upshift_rpm = {},
		downshift_rpm = {},
		throttle_shift_factor = 0,
		kickdown_threshold = math.huge,
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
		min_shift_interval = math.huge,
		gear_ratios = nil,
		final_drive = 1.0,
		wheel_radius = 0.3,
		engine_idle_rpm = 800.0,
		engine_rev_limiter_rpm = math.huge,
		coast_upshift_rpm = 2600.0,
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
	}
	if overrides then for k, v in overrides do t[k] = v end end
	return t
end

-- Update the automatic transmission controller from virtual sensor signals.
-- Speed is supplied in km/h to match the Rust TCM contract. The controller
-- only requests one adjacent gear per update and leaves clutch/shift timing
-- to the drivetrain state machine.
function Drivetrain.updateTCM(tcm, engine_rpm, vehicle_speed_kmh, throttle, brake, grade, temperature, current_gear, max_gear, dt)
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
	local modeProfile = Drivetrain.getDriveModeProfile(tcm.drive_mode)
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
		and tcm.observed_throttle < tcm.lockup_throttle_threshold
		and (brake or 0) < 0.1
		and tcm.observed_input_rpm > 1500 then
		tcm.converter_lockup = true
	elseif vehicle_speed_kmh < speed_threshold * 0.7
		or tcm.observed_throttle > 0.7
		or (brake or 0) > 0.2 then
		tcm.converter_lockup = false
	end

	tcm.shift_latency = Drivetrain.tcmShiftDurationMultiplier(tcm)
	if current_gear <= 0 or current_gear > max_gear then return nil end

	-- Throttle-based economy scaling: low pedal = fuel efficiency →
	-- shift early (lower upshift RPM) and hold gear longer (lower
	-- downshift RPM). At zero throttle upshift at ~70% of authored
	-- RPM; at full throttle shift at authored RPM.
	local throttle = tcm.observed_throttle
	local economy_scale = 1.0 - throttle * 0.3
	local upshift = (tcm.upshift_rpm[index + 1] or math.huge) * economy_scale
		+ (modeProfile.tcm_upshift_modifier or 0)
		+ throttle * tcm.throttle_shift_factor * 1000
		+ math.clamp(tcm.engine_load, 0, 1) * tcm.load_shift_factor
	local downshift = (tcm.downshift_rpm[index + 1] or 0) * economy_scale
		+ (modeProfile.tcm_downshift_modifier or 0)
		- grade * tcm.grade_factor * 300
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
	-- Protect the selected gear from output-shaft over-rev even when the
	-- converter/input sensor is lagging. This prevents a vehicle from staying
	-- in second while road speed keeps climbing.
	if outputOverrev then
		requested = current_gear + 1
	elseif limpActive then
		local failSafeGear = math.clamp(tcm.limp_gear, 1, math.max(max_gear, 1))
		if current_gear < failSafeGear then
			requested = failSafeGear
		elseif current_gear > failSafeGear and safeDownshift(failSafeGear) then
			requested = failSafeGear
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
			+ (nextDownshift - tcm.engine_idle_rpm) * 0.4
		if not projectedNextRPM or projectedNextRPM >= minimumNextRPM then
			requested = nextGear
		end
	elseif current_gear > 1
		and tcm.time_since_shift >= math.max(tcm.min_shift_interval, tcm._power_downshift_delay)
		and tcm.requested_power_kw > tcm.available_power_kw + math.max(5, tcm.available_power_kw * 0.08)
		and safeDownshift(current_gear - 1) then
		requested = current_gear - 1
	elseif current_gear > 1 and tcm.observed_input_rpm < downshift and safeDownshift(current_gear - 1) then
		requested = current_gear - 1
	end

	if requested then
		tcm.pending_gear = requested
		tcm.time_since_shift = 0
	end
	return requested
end

function Drivetrain.tcmShiftDurationMultiplier(tcm)
	local multiplier = 1.0
	local hasFault = function(mask)
		return math.floor(tcm.fault_mask / mask) % 2 == 1
	end
	if hasFault(32) then multiplier *= 2.2 end -- shift solenoid
	if hasFault(64) then multiplier *= 1.6 end -- pressure control
	if hasFault(128) then multiplier *= 1.8 end -- hydraulic pressure
	return math.clamp(multiplier, 1.0, 4.0)
end

function Drivetrain.tcmFailSafeGear(tcm, max_gear)
	return math.clamp(tcm.limp_gear, 1, math.max(max_gear, 1))
end

function Drivetrain.resolveLimpTarget(tcm, target, max_gear)
	if not tcm or not tcm.limp_mode then return target end
	local failSafeGear = Drivetrain.tcmFailSafeGear(tcm, max_gear)
	if not target or target < failSafeGear then return failSafeGear end
	return target
end

-- ============================================================
-- Drive modes
-- ============================================================

local DRIVE_MODE_PROFILES = {
	Normal  = { throttle_map = 1.0, throttle_response = 1.0, idle_rpm_offset = 0, engine_braking_factor = 1.0, steering_assist = 1.0, steering_response = 1.0, suspension_damping = 1.0, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0, tcm_state = "Normal", tcm_upshift_modifier = 0, tcm_downshift_modifier = 0, tcm_lockup_modifier = 0 },
	Eco     = { throttle_map = 0.7, throttle_response = 0.8, idle_rpm_offset = -100, engine_braking_factor = 1.2, steering_assist = 1.2, steering_response = 0.9, suspension_damping = 0.9, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0, tcm_state = "Normal", tcm_upshift_modifier = -500, tcm_downshift_modifier = 200, tcm_lockup_modifier = -10 },
	Comfort = { throttle_map = 0.85, throttle_response = 0.9, idle_rpm_offset = 0, engine_braking_factor = 0.8, steering_assist = 1.3, steering_response = 0.85, suspension_damping = 0.8, diff_mode = "Open", abs_mod = 1.0, tc_mod = 1.0, vsc_mod = 1.0, tcm_state = "Normal", tcm_upshift_modifier = -300, tcm_downshift_modifier = 100, tcm_lockup_modifier = 0 },
	Sport   = { throttle_map = 1.1, throttle_response = 1.2, idle_rpm_offset = 200, engine_braking_factor = 1.3, steering_assist = 0.8, steering_response = 1.2, suspension_damping = 1.2, diff_mode = "LimitedSlip", abs_mod = 1.2, tc_mod = 1.3, vsc_mod = 1.2, tcm_state = "Sport", tcm_upshift_modifier = 400, tcm_downshift_modifier = -200, tcm_lockup_modifier = 10 },
	Track   = { throttle_map = 1.2, throttle_response = 1.3, idle_rpm_offset = 300, engine_braking_factor = 1.5, steering_assist = 0.6, steering_response = 1.3, suspension_damping = 1.5, diff_mode = "Locked", abs_mod = 1.5, tc_mod = 1.5, vsc_mod = 1.5, tcm_state = "Sport", tcm_upshift_modifier = 600, tcm_downshift_modifier = -300, tcm_lockup_modifier = 15 },
	Snow    = { throttle_map = 0.6, throttle_response = 0.7, idle_rpm_offset = 0, engine_braking_factor = 0.7, steering_assist = 1.3, steering_response = 0.7, suspension_damping = 0.7, diff_mode = "LimitedSlip", abs_mod = 0.8, tc_mod = 0.7, vsc_mod = 0.7, tcm_state = "Winter", tcm_upshift_modifier = -600, tcm_downshift_modifier = 300, tcm_lockup_modifier = -15 },
}

function Drivetrain.getDriveModeProfile(mode)
	return DRIVE_MODE_PROFILES[mode] or DRIVE_MODE_PROFILES.Normal
end

function Drivetrain.setDriveMode(d, mode)
	local profile = Drivetrain.getDriveModeProfile(mode)
	local selected = if DRIVE_MODE_PROFILES[mode] then mode else "Normal"
	d.drive_mode = selected
	if d.tcm then
		d.tcm.drive_mode = selected
		d.tcm.state = profile.tcm_state
	end
	if d.differential then d.differential.mode = profile.diff_mode end
	return selected
end

function Drivetrain.mapThrottle(profile, raw_throttle)
	return math.clamp(raw_throttle * profile.throttle_map, 0, 1)
end

return Drivetrain
