--[[

	Engine subsystem — torque curve lookup, RPM integration, idle control.
	Extracted from drivetrain.luau for maintainability.

]]

local Engine = {}

function Engine.new(overrides)
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

function Engine.torqueAtRPMFromCurve(curve, rpm)
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

function Engine.torqueAtRPM(engine, rpm)
	return Engine.torqueAtRPMFromCurve(engine.torque_curve, rpm)
end

function Engine.update(engine, throttle, dt, shaft_rpm, coupling, external_load_fraction)
	local torque = Engine.torqueAtRPM(engine, engine.rpm) * math.clamp(throttle or 0, 0, 1)
	-- A slipping automatic converter still loads the impeller. Without this
	-- reaction torque, the engine can free-rev to the limiter in D at standstill
	-- because only the small shaft-synchronization term remains. Park/Neutral
	-- pass zero load from the caller so the engine can free-rev there.
	local converterLoad = math.clamp(external_load_fraction or 0, 0, 1)
	torque -= torque * converterLoad
	local rpmFraction = math.clamp(engine.rpm / math.max(engine.redline_rpm, 1), 0, 1)
	local friction = engine.engine_braking * (0.25 + rpmFraction * 0.75)
	-- Rigid coupling (locked converter / clutch): snap engine to output
	-- shaft directly. The P-controller below only handles slip cases.
	if type(shaft_rpm) == "number" and shaft_rpm == shaft_rpm
		and (coupling or 0) > 0.95 then
		engine.rpm = shaft_rpm
	else
		local syncTorque = 0
		if type(shaft_rpm) == "number" and shaft_rpm == shaft_rpm then
			local angularError = (shaft_rpm - engine.rpm) * (2 * math.pi / 60)
			local syncGain = math.max(engine.throttle_response, 0.1) * 2.0
			syncTorque = angularError * math.max(engine.inertia, 0.01) * syncGain * math.clamp(coupling or 0, 0, 1)
		end
		local angularAcceleration = (torque - friction + syncTorque) / math.max(engine.inertia, 0.01)
		engine.rpm += angularAcceleration * (60 / (2 * math.pi)) * math.max(dt, 0)
	end
	-- Idle air control: the engine always maintains at least idle RPM when
	-- running. Previously this was gated on throttle > 0.01, which let RPM
	-- decay to zero whenever the driver lifted off — the torque converter
	-- slipping case now has no syncTorque to arrest the decay.
	if engine.rpm < engine.idle_rpm then
		engine.rpm = engine.idle_rpm
	end
	engine.rpm = math.clamp(engine.rpm, 0, engine.rev_limiter_rpm)
end

return Engine
