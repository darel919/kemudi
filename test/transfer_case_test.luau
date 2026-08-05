local Drivetrain = require("../src/vehicle/drivetrain")
local TransferCase = require("../src/vehicle/transfer_case")

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

local function makeGVR()
	return Drivetrain.new({
		engine = {
			idle_rpm = 725,
			redline_rpm = 4500,
			rev_limiter_rpm = 4650,
			stall_rpm = 350,
			inertia = 0.42,
			throttle_response = 1.0,
			engine_braking = 0,
			engine_braking_curve = {
				{ 800, -12 }, { 1200, -25 }, { 1800, -48 },
				{ 2500, -78 }, { 3200, -112 }, { 4000, -145 }, { 4500, -165 },
			},
			accessory_drag_idle = 18,
			accessory_drag_2000 = 34,
			accessory_drag_4000 = 60,
			idle_governor_max_torque = 185,
			torque_curve = {
				{ 600, 80 }, { 725, 145 }, { 900, 235 }, { 1100, 335 },
				{ 1300, 420 }, { 1500, 465 }, { 1700, 484 }, { 2000, 484 },
				{ 2500, 484 }, { 3000, 458 }, { 3500, 438 }, { 3750, 438 },
				{ 4000, 407 }, { 4250, 350 }, { 4500, 285 }, { 4650, 0 },
			},
		},
		transmission = {
			mode = "Automatic",
			gear_ratios = { 4.696, 2.985, 2.146, 1.769, 1.520, 1.275, 1.000, 0.854, 0.689, 0.636 },
			final_drive = 3.73,
			reverse_ratio = -4.866,
			clutch_engagement = 1.0,
		},
		tcm = {
			enabled = true,
			upshift_rpm = { 1700, 1650, 1600, 1550, 1500, 1500, 1450, 1450, 1400, 0 },
			downshift_rpm = { 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 0 },
			throttle_shift_factor = 0.20,
			kickdown_threshold = 0.75,
			lockup_speed_threshold = 45,
			lockup_throttle_threshold = math.huge,
			thermal_limit = 140,
			limp_gear = 2,
			min_shift_interval = 0.65,
		},
		transfer_case = {
			low_range_ratio = 2.64,
			center_coupling_rate = 0.15,
			front_axle_disconnect = true,
			rear_diff_type = "E-Locker",
			rear_locker_max_speed = 30,
			front_diff_type = "Open",
			torque_split_front = 0.5,
			efficiency_2h = 0.92,
			efficiency_4h = 0.88,
			efficiency_4l = 0.85,
		},
		front_differential = { mode = "Open", bias = 0.5 },
		rear_differential = { mode = "Open", bias = 0.5 },
	})
end

test("transfer case initializes in 2H", function()
	local d = makeGVR()
	assert(d.transfer_case.mode == "2H", "default mode should be 2H")
	assert(d.transfer_case.low_range_active == false, "low range should be inactive")
	assert(d.transfer_case.center_coupling == 0, "center coupling should be 0 in 2H")
end)

test("transfer case mode request to 4H", function()
	local d = makeGVR()
	local ok = Drivetrain.transferCaseRequestMode(d.transfer_case, "4H", 0, 1)
	assert(ok == true, "4H request should succeed at stop")
end)

test("transfer case mode request to 4L requires stop", function()
	local d = makeGVR()
	local ok = Drivetrain.transferCaseRequestMode(d.transfer_case, "4L", 50, 0)
	assert(ok == false, "4L request should fail at speed")
	local ok2 = Drivetrain.transferCaseRequestMode(d.transfer_case, "4L", 0, 1)
	assert(ok2 == true, "4L request should succeed at stop with brake")
end)

test("transfer case low range multiplier", function()
	local d = makeGVR()
	local tc = d.transfer_case
	local tc_cfg = d.transfer_case_config
	
	-- 2H: multiplier = 1
	tc.mode = "2H"
	tc.low_range_active = false
	local mult2h = Drivetrain.transferCaseGetLowRangeMult(tc, tc_cfg)
	assert(math.abs(mult2h - 1.0) < 0.001, "2H multiplier should be 1.0, got " .. mult2h)
	
	-- 4L: multiplier = 2.64
	tc.mode = "4L"
	tc.low_range_active = true
	local mult4l = Drivetrain.transferCaseGetLowRangeMult(tc, tc_cfg)
	assert(math.abs(mult4l - 2.64) < 0.001, "4L multiplier should be 2.64, got " .. mult4l)
end)

test("total ratio with transfer case", function()
	local d = makeGVR()
	d.transmission.selector = "D"
	d.transmission.current_gear = 1
	-- 1st gear in 2H: 4.696 * 3.73 = 17.516
	d.transfer_case.mode = "2H"
	d.transfer_case.low_range_active = false
	local r1_2h = Drivetrain.totalRatioWithTC(d)
	assert(math.abs(r1_2h - 17.516) < 0.01, "1st gear 2H ratio mismatch: " .. r1_2h)
	
	-- 1st gear in 4L: 17.516 * 2.64 = 46.24
	d.transfer_case.mode = "4L"
	d.transfer_case.low_range_active = true
	local r1_4l = Drivetrain.totalRatioWithTC(d)
	assert(math.abs(r1_4l - 46.24) < 0.05, "1st gear 4L ratio mismatch: " .. r1_4l)
end)

test("torque split in 2H vs 4H", function()
	local d = makeGVR()
	local tc = d.transfer_case
	local tc_cfg = d.transfer_case_config
	
	-- 2H: all torque to rear
	tc.mode = "2H"
	tc.center_coupling = 0
	local front, rear = Drivetrain.transferCaseSplitTorque(tc, tc_cfg, 1000)
	assert(front == 0, "2H front torque should be 0, got " .. front)
	assert(rear == 1000, "2H rear torque should be 1000, got " .. rear)
	
	-- 4H: 50/50 split
	tc.mode = "4H"
	tc.center_coupling = 1
	front, rear = Drivetrain.transferCaseSplitTorque(tc, tc_cfg, 1000)
	assert(math.abs(front - 500) < 1, "4H front torque should be ~500, got " .. front)
	assert(math.abs(rear - 500) < 1, "4H rear torque should be ~500, got " .. rear)
	
	-- 4A: partial coupling
	tc.mode = "4A"
	tc.center_coupling = 0.5
	front, rear = Drivetrain.transferCaseSplitTorque(tc, tc_cfg, 1000)
	assert(front < 500, "4A front torque should be < 500 at 0.5 coupling, got " .. front)
end)

test("front axle disconnect in 2H", function()
	local d = makeGVR()
	local tc = d.transfer_case
	local tc_cfg = d.transfer_case_config
	
	tc.mode = "2H"
	tc_cfg.front_axle_disconnect = true
	local front_mode = TransferCase.frontDiffMode(tc, tc_cfg)
	assert(front_mode == "Disconnected", "2H with FAD should be Disconnected, got " .. front_mode)
	
	tc.mode = "4H"
	front_mode = TransferCase.frontDiffMode(tc, tc_cfg)
	assert(front_mode == "Open", "4H front should be Open, got " .. front_mode)
end)

test("rear e-locker engagement speed gate", function()
	local d = makeGVR()
	local tc = d.transfer_case
	local tc_cfg = d.transfer_case_config
	
	tc_cfg.rear_diff_type = "E-Locker"
	
	-- Request locker at low speed
	tc.rear_locker_requested = true
	Drivetrain.transferCaseUpdate(tc, tc_cfg, 20, 1/60)
	assert(tc.rear_locker_engaged == true, "locker should engage at 20 km/h")
	
	-- Request locker at high speed
	tc.rear_locker_requested = true
	Drivetrain.transferCaseUpdate(tc, tc_cfg, 50, 1/60)
	assert(tc.rear_locker_engaged == false, "locker should NOT engage at 50 km/h")
end)

test("crawl ratio calculation", function()
	local d = makeGVR()
	d.transmission.selector = "D"
	d.transmission.current_gear = 1
	d.transfer_case.mode = "4L"
	d.transfer_case.low_range_active = true
	
	-- 1st gear in 4L: 4.696 * 3.73 * 2.64 = 46.24:1
	local crawl = Drivetrain.totalRatioWithTC(d)
	assert(math.abs(crawl - 46.24) < 0.1, "crawl ratio should be ~46.24:1, got " .. crawl .. ":1")
	
	-- Speed at 725 rpm in 4L 1st: 2.41 km/h
	local wheel_rpm = 725 / crawl
	local wheel_speed_rad_s = wheel_rpm * 2 * math.pi / 60
	local speed_ms = wheel_speed_rad_s * 0.397
	local speed_kmh = speed_ms * 3.6
	assert(math.abs(speed_kmh - 2.41) < 0.1, "idle crawl speed should be ~2.41 km/h, got " .. speed_kmh)
end)

print(string.format("\n=== Transfer Case Tests: %d/%d passed ===", passed, total))

if failed > 0 then
	error(failed .. " tests failed")
end