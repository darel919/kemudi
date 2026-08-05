--[[
	Drivetrain — engine, transmission, differential, TCM, drive modes.
	Ported from crates/kemudi-engine/src/drivetrain/*.ss
	Pure Luau, no Roblox API dependencies.
--]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")
local EngineSub = if script then require(script.Parent.drivetrain_engine) else require("./drivetrain_engine")
local TCMSub = if script then require(script.Parent.drivetrain_tcm) else require("./drivetrain_tcm")
local TransferCase = if script then require(script.Parent.transfer_case) else require("./transfer_case")

local Drivetrain = {}

-- ============================================================
-- Engine
-- ============================================================

-- Engine functions (extracted to drivetrain/engine.luau)
Drivetrain.newEngine = EngineSub.new
Drivetrain.torqueAtRPMFromCurve = EngineSub.torqueAtRPMFromCurve
Drivetrain.engineTorqueAtRPM = EngineSub.torqueAtRPM
Drivetrain.updateEngine = EngineSub.update

-- Transfer case functions
Drivetrain.newTransferCase = TransferCase.newState
Drivetrain.transferCaseConfig = TransferCase.newConfig
Drivetrain.transferCaseRequestMode = TransferCase.requestMode
Drivetrain.transferCaseUpdate = TransferCase.update
Drivetrain.transferCaseGetLowRangeMult = TransferCase.getLowRangeMult
Drivetrain.transferCaseSplitTorque = TransferCase.splitTorque
Drivetrain.transferCaseGetEfficiency = TransferCase.getEfficiency
Drivetrain.transferCaseFrontDriveAmount = TransferCase.frontDriveAmount
Drivetrain.transferCaseRearDiffMode = TransferCase.rearDiffMode
Drivetrain.transferCaseFrontDiffMode = TransferCase.frontDiffMode
Drivetrain.transferCaseTelemetry = TransferCase.telemetry


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
		-- Normalize to negative regardless of whether the asset authors
		-- reverse_ratio as positive (natural magnitude) or negative.
		return -math.abs(trans.reverse_ratio) * trans.final_drive
	end
	return 0
end

-- Total ratio including transfer case low range (if active)
function Drivetrain.totalRatioWithTC(d)
	local base = Drivetrain.totalRatio(d.transmission)
	if base == 0 then return 0 end
	local tcMult = 1.0
	if d.transfer_case and d.transfer_case_config then
		tcMult = Drivetrain.transferCaseGetLowRangeMult(d.transfer_case, d.transfer_case_config)
	end
	return base * tcMult
end

function Drivetrain.engineRPMFromWheelSpeed(trans, wheel_speed)
	local ratio = math.abs(Drivetrain.totalRatio(trans))
	return math.abs(wheel_speed) * ratio * 60 / (2 * math.pi)
end

-- Engine RPM from wheel speed including transfer case
function Drivetrain.engineRPMFromWheelSpeedTC(d, wheel_speed)
	local ratio = math.abs(Drivetrain.totalRatioWithTC(d))
	return math.abs(wheel_speed) * ratio * 60 / (2 * math.pi)
end

function Drivetrain.wheelTorque(trans, engine_torque)
	return engine_torque * Drivetrain.totalRatio(trans) * trans.clutch_engagement
end

function Drivetrain.wheelTorqueTC(d, engine_torque)
	return engine_torque * Drivetrain.totalRatioWithTC(d) * d.transmission.clutch_engagement
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
		differential = Drivetrain.newDifferential(), -- legacy single diff (RWD/FWD)
		front_differential = Drivetrain.newDifferential(), -- AWD: front axle diff
		rear_differential = Drivetrain.newDifferential(),  -- AWD: rear axle diff
		transfer_case = Drivetrain.newTransferCase(overrides.transfer_case),
		transfer_case_config = Drivetrain.transferCaseConfig(overrides.transfer_case),
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
			if k ~= "engine" and k ~= "transmission" and k ~= "differential" and k ~= "tcm" and k ~= "transfer_case" then
				d[k] = v
			end
		end
		if overrides.engine then for k, v in overrides.engine do d.engine[k] = v end end
		if overrides.transmission then for k, v in overrides.transmission do d.transmission[k] = v end end
		if overrides.differential then for k, v in overrides.differential do d.differential[k] = v end end
		if overrides.front_differential then for k, v in overrides.front_differential do d.front_differential[k] = v end end
		if overrides.rear_differential then for k, v in overrides.rear_differential do d.rear_differential[k] = v end end
		if overrides.transfer_case then for k, v in overrides.transfer_case do d.transfer_case[k] = v end end
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
			local postShiftCoupled = Drivetrain.engineRPMFromWheelSpeedTC(d, d.wheel_speed)
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

	-- Per-gear shift duration from TCM (overrides base shift_duration)
	if d.tcm and d.tcm.shift_duration and d.tcm.shift_duration[d.transmission.current_gear] then
		d.shift_duration = d.tcm.shift_duration[d.transmission.current_gear]
	end

	-- Shift torque reduction from TCM (applied during shift phases)
	local shiftTorqueReduction = 0
	if d.tcm and d.tcm.shift_torque_reduction and d.tcm.shift_torque_reduction[d.transmission.current_gear] then
		shiftTorqueReduction = d.tcm.shift_torque_reduction[d.transmission.current_gear]
	end
	-- Apply torque reduction during active shift phases
	if shift.phase == "Disengaging" or shift.phase == "Neutral" or shift.phase == "Engaging" then
		if shiftTorqueReduction > 0 then
			d.tcm.torque_reduction_request = math.max(d.tcm.torque_reduction_request or 0, shiftTorqueReduction)
		end
	else
		-- Clear shift torque reduction when shift completes
		if d.tcm and d.tcm.torque_reduction_request and d.tcm.torque_reduction_request > 0 then
			-- Keep TCM-requested reduction (e.g. for power downshifts)
			if d.tcm.torque_reduction_request < shiftTorqueReduction then
				d.tcm.torque_reduction_request = 0
			end
		end
	end

	-- Rigid coupling detection
	local coupled_rpm = Drivetrain.engineRPMFromWheelSpeedTC(d, d.wheel_speed)
	local rigid_coupling = d.transmission.clutch_engagement > 0.95
		and (d.transmission.mode == "Manual" or d.converter_coupling > 0.95)
		and (math.abs(d.wheel_speed) > 0.5 or d.engine.rpm >= d.engine.idle_rpm * 0.7)

	-- ISC (Idle Speed Control): maintain idle RPM when converter loads engine.
		-- Two components: base idle throttle (minimum in gear for creep) and
		-- RPM correction (proportional to deficit below idle target). Without
		-- this, zero throttle → zero engine torque → no creep in D/R.
		-- The idle governor max torque cap (configurable per-asset) prevents the
		-- ISC from commanding full engine torque when throttle is released.
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
			-- Idle governor max torque cap: the ISC must not command more than
			-- the authorend limit (e.g. 185 N·m for the GVR D30T). When the ISC
			-- is the only throttle source, clamp engine torque to this limit.
			local govMax = d.engine.idle_governor_max_torque or 0
			if govMax > 0 and d.throttle_input < 0.05 and not isParked then
				engine_torque = math.min(engine_torque, govMax)
			end
		end
	d.last_engine_torque = engine_torque
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

	-- Transfer case update (early, before torque calculation)
	if d.transfer_case and d.transfer_case_config then
		Drivetrain.transferCaseUpdate(d.transfer_case, d.transfer_case_config, d._vehicle_speed_kmh or 0, dt)
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
	-- Use total ratio including transfer case
	local drive_torque = Drivetrain.wheelTorqueTC(d, engine_torque)
		* converter_factor
		* math.clamp(d.transmission.torque_derate or 1, 0, 1)
	-- Apply transfer case efficiency
	if d.transfer_case and d.transfer_case_config then
		local tcEff = Drivetrain.transferCaseGetEfficiency(d.transfer_case, d.transfer_case_config)
		drive_torque = drive_torque * tcEff
	end

	-- Drivetrain efficiency model (GVR spec §5.1)
	-- Locked converter: 88-92%, Open converter: 72-88%
	-- This models losses in driveshafts, differentials, bearings, etc.
	-- Values come from vehicle asset config (efficiency_open, efficiency_locked, efficiency_manual)
	local drivetrain_eff
	if d.transmission.mode == "Automatic" then
		-- Interpolate between open and locked based on coupling
		local coupling = d.converter_coupling or 0
		local eff_open = d.efficiency_open or 0.80  -- midpoint of 72-88%
		local eff_locked = d.efficiency_locked or 0.90  -- midpoint of 88-92%
		drivetrain_eff = eff_open + (eff_locked - eff_open) * coupling
	else
		-- Manual transmission: fixed efficiency
		drivetrain_eff = d.efficiency_manual or 0.88
	end
	drive_torque = drive_torque * drivetrain_eff

	-- Angular integration
	local wheel_inertia = Drivetrain.effectiveWheelInertia(dt, vehicle_mass, d.wheel_inertia)
	local stopping_torque = math.abs(d.wheel_speed) * wheel_inertia / math.max(dt, 1e-6)

	-- Engine braking
	local requested_engine_brake = 0
	if effective_throttle < 0.05 and math.abs(d.wheel_speed) > 1e-6 and d.transmission.clutch_engagement > 0.05 then
		requested_engine_brake = d.engine.engine_braking * math.abs(Drivetrain.totalRatioWithTC(d))
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

-- TCM functions (extracted to drivetrain/tcm.luau)
Drivetrain.newTCM = TCMSub.new
Drivetrain.tcmShiftDurationMultiplier = TCMSub.shiftDurationMultiplier
Drivetrain.tcmFailSafeGear = TCMSub.failSafeGear
Drivetrain.resolveLimpTarget = TCMSub.resolveLimpTarget
Drivetrain.getDriveModeProfile = TCMSub.getDriveModeProfile
Drivetrain.setDriveMode = TCMSub.setDriveMode
Drivetrain.mapThrottle = TCMSub.mapThrottle

function Drivetrain.updateTCM(tcm, engine_rpm, vehicle_speed_kmh, throttle, brake, grade, temperature, current_gear, max_gear, dt)
	return TCMSub.update(tcm, engine_rpm, vehicle_speed_kmh, throttle, brake, grade, temperature, current_gear, max_gear, dt, Drivetrain)
end

return Drivetrain