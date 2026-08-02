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

test("coasting stops shifting when the projected RPM is efficient", function()
	local d = makeDrivetrain()
	d.wheel_radius = 0.438
	d.tcm.wheel_radius = d.wheel_radius
	local target = Drivetrain.updateTCM(d.tcm, 2100, 123, 0, 0, 0, 20, 7, 10, 1 / 60)
	assert(target == nil, "gear 7 should be retained near the coast RPM target")
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
	d.tcm.time_since_shift = 1
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
	d.converter_coupling = 1
	d.throttle_input = 0
	d.wheel_speed = 50 / 0.438
	for _ = 1, 10 do
		Drivetrain.update(1 / 60, 1011, d)
	end
	assert(d.engine.rpm > 3000, "locked shaft should load/couple the engine above idle")
	assert(d.engine.rpm <= d.engine.rev_limiter_rpm, "shaft coupling must respect the rev limiter")
end)

print(string.format("=== Drivetrain TCM regression: %d/%d passed ===", passed, total))
if failed > 0 then
	error(string.format("%d drivetrain regression tests failed", failed))
end