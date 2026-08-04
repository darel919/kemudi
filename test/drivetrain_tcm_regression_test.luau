local Drivetrain = require("../src/vehicle/drivetrain")

local passed = 0
local failed = 0
local total = 0

local function test(name, callback)
	total += 1
	local ok, message = pcall(callback)
	if ok then
		passed += 1
		print("PASS", name)
	else
		failed += 1
		warn("FAIL", name, message)
	end
end

local function makeDrivetrain()
	return Drivetrain.new({
		engine = {
			idle_rpm = 700,
			redline_rpm = 6500,
			rev_limiter_rpm = 6800,
			throttle_response = 3.2,
			engine_braking = 42,
			inertia = 0.32,
			torque_curve = {{700, 300}, {1200, 360}, {2000, 430}, {3000, 475}, {4000, 500}, {5000, 470}, {6000, 410}, {6500, 340}},
		},
		transmission = {
			mode = "Automatic",
			gear_ratios = {4.70, 2.99, 2.15, 1.77, 1.52, 1.28, 1.00, 0.85, 0.69, 0.64},
			final_drive = 3.15,
			reverse_ratio = -4.87,
			clutch_engagement = 1,
		},
		tcm = {
			enabled = true,
			upshift_rpm = {5600, 5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000, 0},
			downshift_rpm = {1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 0},
			throttle_shift_factor = 0.20,
			kickdown_threshold = 0.78,
			lockup_speed_threshold = 45,
			lockup_throttle_threshold = 0.30,
			thermal_limit = 140,
			limp_gear = 2,
			min_shift_interval = 0.65,
		},
	})
end

test("coasting requests the next efficient gear from output speed", function()
	local d = makeDrivetrain()
	d.transmission.current_gear = 2
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	local target = Drivetrain.updateTCM(d.tcm, 1400, 123, 0, 0, 0, 20, 2, 10, 1 / 60)
	assert(target == 3, "expected coast upshift from gear 2 to gear 3")
end)

test("automatic transmission initializes in Park", function()
	local d = makeDrivetrain()
	assert(d.transmission.selector == "P", "automatic transmission must start in P")
	assert(d.transmission.current_gear == 0, "Park must not select a forward gear")
end)

test("selector advances and reverses one position at a time", function()
	local d = makeDrivetrain()
	local function completeSelectorShift()
		if d.shift.phase ~= "Idle" then d.transmission.current_gear = d.shift.pending_gear end
		d.shift.pending_gear = 0
		d.shift.phase = "Idle"
	end
	assert(Drivetrain.requestSelectorUp(d, 0, 1, false), "P to R should be permitted while stopped with brake")
	assert(d.transmission.selector == "R", "selector must advance from P to R")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorUp(d, 0, 1, false), "R to N should be permitted")
	assert(d.transmission.selector == "N", "selector must advance from R to N")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorUp(d, 0, 1, false), "N to D should be permitted while stopped with brake")
	assert(d.transmission.selector == "D", "selector must advance from N to D")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorUp(d, 0, 1, false), "D to M/S should be permitted")
	assert(d.transmission.selector == "M/S", "selector must advance from D to M/S")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorDown(d, 0, 1, false), "M/S to D should be permitted")
	assert(d.transmission.selector == "D", "selector must reverse from M/S to D")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorDown(d, 0, 1, false), "D to N should be permitted")
	assert(d.transmission.selector == "N", "selector must reverse from D to N")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorDown(d, 0, 1, false), "N to R should be permitted while stopped")
	assert(d.transmission.selector == "R", "selector must reverse from N to R")
	completeSelectorShift()
	assert(Drivetrain.requestSelectorDown(d, 0, 1, false), "R to P should be permitted while stopped")
	assert(d.transmission.selector == "P", "selector must reverse from R to P")
end)

test("selector rejects direction changes while moving without changing state", function()
	local d = makeDrivetrain()
	local ok = Drivetrain.requestSelectorUp(d, 12, 1, true)
	assert(not ok, "P to R must be rejected while moving")
	assert(d.transmission.selector == "P", "rejected selector request must not change state")
	assert(d.transmission.last_rejection == "vehicle_moving", "rejection reason must be observable")
end)

test("selector detents during an active shift are ignored without an alert", function()
	local d = makeDrivetrain()
	assert(Drivetrain.requestSelectorUp(d, 0, 1, true), "initial P to R request should succeed")
	assert(d.shift.phase ~= "Idle", "initial selector request should start a shift")
	d.transmission.last_rejection = nil
	assert(not Drivetrain.requestSelectorUp(d, 0, 1, true), "second detent must wait for the active shift")
	assert(d.transmission.selector == "R", "ignored detent must not advance the selector")
	assert(d.transmission.last_rejection == nil, "normal detent timing must not emit a rejection alert")
end)

test("neutral to drive while moving is allowed but records engagement damage risk", function()
	local d = makeDrivetrain()
	d.transmission.selector = "N"
	d.transmission.current_gear = 0
	d.wheel_radius = 0.438
	d.wheel_speed = (100 / 3.6) / d.wheel_radius
	d.throttle_input = 1
	assert(Drivetrain.requestSelectorUp(d, 100, 0, true), "N to D should remain operable without brake input")
	assert(d.transmission.selector == "D", "N to D must select Drive")
	assert(d.transmission.pending_shift_damage > 0, "moving N to D must record damage risk")
	Drivetrain.update(1 / 60, 1011, d)
	assert(d.transmission.damage > 0, "moving N to D must accumulate transmission damage")
end)

test("neutral to reverse while moving is allowed but records severe engagement risk", function()
	local d = makeDrivetrain()
	d.transmission.selector = "N"
	d.transmission.current_gear = 0
	d.wheel_radius = 0.438
	d.wheel_speed = (100 / 3.6) / d.wheel_radius
	d.throttle_input = 1
	assert(Drivetrain.requestSelectorDown(d, 100, 0, true), "N to R should remain operable without brake input")
	assert(d.transmission.selector == "R", "N to R must select Reverse")
	assert(d.transmission.pending_shift_damage > 0, "moving N to R must record damage risk")
	Drivetrain.update(1 / 60, 1011, d)
	assert(d.transmission.damage > 0, "moving N to R must accumulate transmission damage")
end)

test("manual M/S shift is accepted only when TCM projected RPM is safe", function()
	local d = makeDrivetrain()
	d.transmission.selector = "M/S"
	d.transmission.current_gear = 2
	d.tcm.time_since_shift = 1
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	d.wheel_speed = 30 / 0.438
	assert(Drivetrain.requestManualShift(d, 1, 108, 0), "safe manual upshift must be accepted")
	assert(d.shift.pending_gear == 3, "manual command must go through the shift state machine")
	local rejected = Drivetrain.requestManualShift(d, -1, 260, 0)
	assert(not rejected, "unsafe manual downshift must be rejected")
	assert(d.transmission.last_rejection == "projected_overrev", "unsafe downshift must expose the TCM rejection")
end)

test("coasting stops shifting when the projected RPM is efficient", function()
	local d = makeDrivetrain()
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	-- At 50 km/h in gear 7, shaft RPM ≈ 954 < coast_upshift_rpm(1200) → no shift
	local target = Drivetrain.updateTCM(d.tcm, 2100, 50, 0, 0, 0, 20, 7, 10, 1 / 60)
	assert(target == nil, "gear 7 should be retained when shaft RPM is below coast threshold")
end)

test("coast upshift engages at highway speed", function()
	local d = makeDrivetrain()
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	d.tcm.time_since_shift = 1
	-- At 123 km/h in gear 7, shaft RPM ≈ 2353 > coast_upshift_rpm(1200) → upshift
	local target = Drivetrain.updateTCM(d.tcm, 2100, 123, 0, 0, 0, 20, 7, 10, 1 / 60)
	assert(target == 8, "gear 8 should be selected at highway coast speed")
end)

test("output speed protection upshifts even when converter RPM is lagging", function()
	local d = makeDrivetrain()
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	d.tcm.time_since_shift = 1
	local target = Drivetrain.updateTCM(d.tcm, 3000, 260, 0.45, 0, 0, 20, 2, 10, 1 / 60)
	assert(target == 3, "gear 2 must not remain selected when output speed would force an over-rev")
end)

test("output over-rev protection bypasses the normal shift interval", function()
	local d = makeDrivetrain()
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	d.tcm.time_since_shift = 0.1
	local target = Drivetrain.updateTCM(d.tcm, 3000, 260, 1.0, 0, 0, 20, 2, 10, 1 / 60)
	assert(target == 3, "safety upshift must not wait for the normal shift interval")
end)

test("limp mode still allows an output-safe upshift above the fail-safe gear", function()
	local d = makeDrivetrain()
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	d.tcm.limp_mode = true
	d.tcm.state = "Limp"
	d.tcm.time_since_shift = 0.1
	local target = Drivetrain.updateTCM(d.tcm, 6800, 260, 1.0, 0, 0, 20, 2, 10, 1 / 60)
	assert(target == 3, "limp mode must upshift when fail-safe gear 2 would overspeed the output shaft")
end)

test("vehicle limp target normalization preserves a safety upshift", function()
	local d = makeDrivetrain()
	d.tcm.limp_mode = true
	d.tcm.limp_gear = 2
	assert(Drivetrain.resolveLimpTarget(d.tcm, nil, 10) == 2, "limp mode must select the fail-safe floor when TCM has no target")
	assert(Drivetrain.resolveLimpTarget(d.tcm, 1, 10) == 2, "limp mode must not select below the fail-safe floor")
	assert(Drivetrain.resolveLimpTarget(d.tcm, 3, 10) == 3, "limp mode must preserve an output-safe upshift")
end)

test("engine power demand requests a downshift before the RPM threshold", function()
	local d = makeDrivetrain()
	d.tcm.time_since_shift = 3
	d.tcm.kickdown_threshold = 1.0
	d.tcm.downshift_rpm[5] = 1000
	d.tcm.torque_curve = d.engine.torque_curve
	d.tcm.peak_torque = 500
	local target = Drivetrain.updateTCM(d.tcm, 2000, 60, 0.95, 0, 0, 20, 5, 10, 1 / 60)
	assert(target == 4, "TCM must downshift when requested power exceeds available engine power")
end)

test("drive mode changes the authored shift strategy", function()
	local d = makeDrivetrain()
	d.tcm.time_since_shift = 1
	d.tcm.drive_mode = "Eco"
	local target = Drivetrain.updateTCM(d.tcm, 5000, 100, 0.2, 0, 0, 20, 2, 10, 1 / 60)
	assert(target == 3, "Eco mode should upshift earlier than Normal mode")
end)

test("throttle input uses engine inertia instead of a direct redline target", function()
	local d = makeDrivetrain()
	d.transmission.current_gear = 1
	d.converter_coupling = 0.25
	d.throttle_input = 1
	d.wheel_speed = 0
	Drivetrain.update(1 / 60, 1011, d)
	assert(d.engine.rpm > d.engine.idle_rpm, "engine should accelerate")
	assert(d.engine.rpm < 2500, "engine RPM should not jump directly toward redline")
end)

test("locked transmission pulls engine RPM toward the shaft speed", function()
	local d = makeDrivetrain()
	d.transmission.current_gear = 2
	d.transmission.selector = "D"
	d.converter_coupling = 1
	d.throttle_input = 0
	d.wheel_speed = 50 / 0.438
	for _ = 1, 10 do
		Drivetrain.update(1 / 60, 1011, d)
	end
	assert(d.engine.rpm > 3000, "locked shaft should load/couple the engine above idle")
	assert(d.engine.rpm <= d.engine.rev_limiter_rpm, "shaft coupling must respect the rev limiter")
end)

test("automatic converter load prevents D-range WOT free-rev at standstill", function()
	local d = makeDrivetrain()
	d.transmission.selector = "D"
	d.transmission.current_gear = 1
	d.engine.rpm = d.engine.rev_limiter_rpm
	d.throttle_input = 1
	d.wheel_speed = 0
	d.converter_coupling = 0.05 -- converter at stall/maximum slip
	Drivetrain.update(1 / 60, 1011, d)
	assert(d.engine.rpm < d.engine.rev_limiter_rpm - 1, "D-range converter load must pull engine away from the limiter")
end)

test("automatic upshift pulls engine RPM down through the new ratio", function()
	local d = makeDrivetrain()
	d.transmission.selector = "D"
	d.transmission.current_gear = 1
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	local wheelSpeed = (70 / 3.6) / d.wheel_radius
	d.wheel_speed = wheelSpeed
	d.engine.rpm = d.engine.rev_limiter_rpm
	d.throttle_input = 1
	d.converter_coupling = 0.95
	local target = Drivetrain.updateTCM(d.tcm, d.engine.rpm, 70, 1, 0, 0, 20, 1, 10, 1 / 60)
	assert(target == 2, "TCM must command the next gear at the limiter")
	assert(Drivetrain.requestShiftUp(d), "TCM upshift must enter the shift state machine")
	for _ = 1, 20 do
		Drivetrain.update(1 / 60, 1011, d)
		-- Hold road speed fixed to isolate ratio-to-engine coupling.
		d.wheel_speed = wheelSpeed
	end
	assert(d.transmission.current_gear == 2, "requested upshift must complete")
	assert(d.engine.rpm < d.engine.rev_limiter_rpm - 1, "upshift must lower engine RPM instead of holding the limiter")
end)
test("normal transmission operation does not accumulate mechanical damage", function()
	local d = makeDrivetrain()
	d.transmission.selector = "D"
	d.transmission.current_gear = 2
	d.transmission_temperature = 80
	d.last_drive_torque = 450
	d.clutch_shock.torque_spike = 0
	for _ = 1, 600 do
		Drivetrain.updateTransmissionHealth(d, 1 / 60)
	end
	assert(d.transmission.damage == 0, "normal temperature and torque must not damage the transmission")
	assert(d.transmission.failure_state == "Healthy", "normal operation must remain healthy")
end)

test("TOC settles transmission near coolant temperature during normal cruise", function()
	local d = makeDrivetrain()
	d.transmission.selector = "D"
	d.transmission.current_gear = 5
	d.wheel_radius = 0.438
	d.converter_coupling = 0.95
	d.throttle_input = 1
	d._toc_coolant_temp = 90 -- warmed-up engine coolant
	d._vehicle_speed_kmh = 80
	d.transmission_temperature = 88 -- warmed-up transmission
	local cruiseWheelSpeed = (80.5 / 3.6) / d.wheel_radius
	d.wheel_speed = cruiseWheelSpeed
	for _ = 1, 600 do
		Drivetrain.update(1 / 60, 1011, d)
		d.wheel_speed = cruiseWheelSpeed
	end
	-- With a working TOC, ATF settles a few degrees above coolant.
	assert(d.transmission_temperature > 85, "TOC must warm transmission toward coolant temperature")
	assert(d.transmission_temperature < 110, "TOC must keep transmission within 20 C of coolant during normal driving")
	assert(d.transmission.damage == 0, "normal cruise with TOC must not damage the transmission")
end)

test("TOC prevents overheating under heavy sustained load", function()
	local d = makeDrivetrain()
	d.transmission.selector = "D"
	d.transmission.current_gear = 2 -- tall load in low gear
	d.wheel_radius = 0.438
	d.converter_coupling = 0.80 -- significant slip
	d.throttle_input = 1
	d._toc_coolant_temp = 90
	d._vehicle_speed_kmh = 60
	d.transmission_temperature = 88
	local wheelSpeed = (60 / 3.6) / d.wheel_radius
	d.wheel_speed = wheelSpeed
	for _ = 1, 2400 do -- 40 seconds of sustained heavy load
		Drivetrain.update(1 / 60, 1011, d)
		d.wheel_speed = wheelSpeed
	end
	assert(d.transmission_temperature < 140, "TOC must prevent transmission from reaching thermal limit under sustained load")
	assert(d.transmission.damage == 0, "TOC-cooled heavy load must not damage the transmission")
end)

test("fall-back to ambient when no engine thermal model is present", function()
	local d = makeDrivetrain()
	d.transmission.selector = "D"
	d.transmission.current_gear = 5
	d.wheel_radius = 0.438
	d.converter_coupling = 0.95
	d.throttle_input = 1
	d._toc_coolant_temp = nil -- no thermal model
	d._vehicle_speed_kmh = 80
	local cruiseWheelSpeed = (80.5 / 3.6) / d.wheel_radius
	d.wheel_speed = cruiseWheelSpeed
	for _ = 1, 600 do
		Drivetrain.update(1 / 60, 1011, d)
		d.wheel_speed = cruiseWheelSpeed
	end
	assert(d.transmission.damage == 0, "fallback-to-ambient cruise must not damage the transmission")
end)

test("repeated clutch abuse accumulates transmission damage and derates torque", function()
	local d = makeDrivetrain()
	d.transmission.selector = "M/S"
	d.transmission.current_gear = 1
	d.transmission_temperature = 130
	d.clutch_shock.torque_spike = 5000
	d.wheel_speed = 25 / 0.438
	for _ = 1, 600 do
		Drivetrain.updateTransmissionHealth(d, 1 / 60)
	end
	assert(d.transmission.damage > 0.2, "repeated clutch shock must accumulate damage")
	assert(d.transmission.failure_state == "Damaged" or d.transmission.failure_state == "Severe", "damaged transmission must expose a degraded state")
	assert(d.transmission.torque_derate < 1, "damage must reduce transmitted torque")
end)

test("extreme transmission abuse causes a mechanical failure and neutralizes the drivetrain", function()
	local d = makeDrivetrain()
	d.transmission.selector = "M/S"
	d.transmission.current_gear = 1
	d.transmission_temperature = 190
	d.clutch_shock.torque_spike = 12000
	d.wheel_speed = 80 / 0.438
	for _ = 1, 600 do
		Drivetrain.updateTransmissionHealth(d, 1 / 60)
	end
	assert(d.transmission.is_failed, "extreme sustained abuse must fail the transmission")
	assert(d.transmission.failure_state == "Failed", "failed transmission must expose Failed state")
	assert(d.transmission.selector == "N", "failed transmission must drop to neutral")
	assert(d.transmission.current_gear == 0, "failed transmission must have no selected gear")
	assert(d.transmission.torque_derate == 0, "failed transmission must transmit no torque")
	assert(d.tcm.state == "Fault", "transmission failure must put the TCM into Fault")
	assert(not Drivetrain.requestManualShift(d, 1, 0, 0), "failed transmission must reject manual commands")
end)

print(string.format("=== Drivetrain TCM regression: %d/%d passed ===", passed, total))
if failed > 0 then
	error(string.format("%d drivetrain regression tests failed", failed))
end