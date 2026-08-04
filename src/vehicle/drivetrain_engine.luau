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
		-- Engine braking torque curve: optional { {rpm, Nm}, ... } table.
		-- When provided, replaces the scalar engine_braking with RPM-dependent lookup.
		-- Negative values represent braking torque (resisting engine rotation).
		engine_braking_curve = overrides.engine_braking_curve,
		-- Accessory drag: torque consumed by alternator, water pump, etc.
		-- Configurable per RPM for realistic parasitic drag (e.g. 18 Nm at idle).
		accessory_drag_idle = overrides.accessory_drag_idle or 0,
		accessory_drag_2000 = overrides.accessory_drag_2000 or 0,
		accessory_drag_4000 = overrides.accessory_drag_4000 or 0,
		-- Idle governor: max torque the ISC can command (prevents runaway creep).
		-- Default of 0 means "no cap" (backward-compatible).
		idle_governor_max_torque = overrides.idle_governor_max_torque or 0,
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

	-- Engine braking: use curve if authored, otherwise fall back to scalar.
	-- The braking curve returns negative torque (resisting rotation).
	local rpmFraction = math.clamp(engine.rpm / math.max(engine.redline_rpm, 1), 0, 1)
	local friction
	if engine.engine_braking_curve and #engine.engine_braking_curve > 0 then
		friction = Engine.torqueAtRPMFromCurve(engine.engine_braking_curve, engine.rpm)
		-- Curves store negative values; ensure we don't accidentally add power.
		friction = math.min(friction, 0)
	else
		friction = -(engine.engine_braking or 0) * (0.25 + rpmFraction * 0.75)
	end

	-- Accessory drag: linearly interpolated from authorend RPM breakpoints.
	-- Models alternated/power pump/fan/etc. parasitic load.
	local accDrag = 0
	if engine.accessory_drag_idle > 0 or engine.accessory_drag_4000 > 0 then
		local dr = engine.accessory_drag_idle or 0
		local d2k = engine.accessory_drag_2000 or dr
		local d4k = engine.accessory_drag_4000 or d2k
		if d4k > 0 or dr > 0 then
			if engine.rpm >= 4000 then
				accDrag = d4k
			elseif engine.rpm >= 2000 then
				local t2 = (engine.rpm - 2000) / 2000
				accDrag = d2k + t2 * (d4k - d2k)
			else
				local t1 = engine.rpm / math.max(1, 2000)
				accDrag = dr + t1 * (d2k - dr)
			end
		end
	end

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
		local angularAcceleration = (torque + friction - accDrag + syncTorque) / math.max(engine.inertia, 0.01)
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
