local Drivetrain = require("../src/vehicle/drivetrain")

local engine = {
	idle_rpm = 800,
	redline_rpm = 6500,
	rev_limiter_rpm = 7000,
	throttle_response = 0.8,
	engine_braking = 30,
	inertia = 0.32,
	torque_curve = { { 800, 300 }, { 3000, 450 }, { 6000, 350 } },
}

local generic = Drivetrain.new({
	engine = engine,
	transmission = {
		mode = "Automatic",
		gear_ratios = { 3.5, 2.1 },
		final_drive = 3.7,
		reverse_ratio = -3.2,
		clutch_engagement = 1.0,
	},
})
assert(generic.tcm.enabled == false, "TCM must not invent calibration without vehicle data")

local authored = Drivetrain.new({
	engine = engine,
	transmission = {
		mode = "Automatic",
		gear_ratios = { 3.8, 2.1, 1.3 },
		final_drive = 4.0,
		reverse_ratio = -3.0,
	},
	tcm = {
		enabled = true,
		upshift_rpm = { 3600, 4200, 5000 },
		downshift_rpm = { 1400, 1600, 1800 },
		throttle_shift_factor = 0.25,
		kickdown_threshold = 0.8,
		lockup_speed_threshold = 50,
		lockup_throttle_threshold = 0.25,
		thermal_limit = 120,
		limp_gear = 2,
		min_shift_interval = 0.5,
	},
})

local request = Drivetrain.updateTCM(authored.tcm, 4100, 30, 0.2, 0, 0, 80, 1, 3, 0.6)
assert(request == 2, "authored upshift map must request the next gear")
assert(authored.tcm.converter_lockup == false, "converter must remain unlocked below its authored speed")

Drivetrain.updateTCM(authored.tcm, 2200, 60, 0.1, 0, 0, 80, 2, 3, 0.6)
assert(authored.tcm.converter_lockup == true, "converter must lock at the authored speed/load point")

-- Full throttle must not select a gear that drops the engine near idle.
authored.tcm.gear_ratios = { 4.70, 2.99, 2.15, 1.77, 1.52, 1.28, 1.00, 0.85, 0.69, 0.64 }
authored.tcm.final_drive = 3.15
authored.tcm.wheel_radius = 0.30
authored.tcm.engine_idle_rpm = 700
authored.tcm.upshift_rpm = { 5000, 5200, 5200, 5000, 4800, 4600, 4400, 4200, 4000, 0 }
authored.tcm.downshift_rpm = { 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2500 }
authored.tcm.time_since_shift = 1
local launchGear = Drivetrain.updateTCM(authored.tcm, 6000, 40, 1.0, 0, 0, 80, 1, 10, 0.1)
assert(launchGear == 2, "full throttle at 40 km/h must request the next useful gear, not top gear")
authored.tcm.time_since_shift = 1
local luggingGear = Drivetrain.updateTCM(authored.tcm, 5000, 40, 1.0, 0, 0, 80, 9, 10, 0.1)
assert(luggingGear == nil, "TCM must reject an upshift whose projected RPM would lug the engine")

local limp = Drivetrain.new({
	engine = engine,
	transmission = {
		mode = "Automatic",
		gear_ratios = { 3.5, 2.1, 1.4, 1.0 },
		final_drive = 3.7,
		reverse_ratio = -3.2,
		clutch_engagement = 1.0,
	},
	tcm = {
		enabled = true,
		upshift_rpm = { 4000 },
		downshift_rpm = { 1500 },
		throttle_shift_factor = 0,
		kickdown_threshold = 0.8,
		lockup_speed_threshold = 40,
		lockup_throttle_threshold = 0.3,
		thermal_limit = 100,
		limp_gear = 2,
		min_shift_interval = 0.5,
	},
})
Drivetrain.updateTCM(limp.tcm, 3000, 40, 0.2, 0, 0, 110, 3, 4, 0.1)
assert(limp.tcm.limp_mode and limp.tcm.torque_reduction_request > 0, "thermal fault must enter authored limp behavior")

print("TCM data-driven tests passed")
