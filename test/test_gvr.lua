-- GVR G-150 Vehicle Asset Specification
-- Full-size, body-on-frame, four-wheel-drive pickup
-- Based on GVR G-150 Vehicle Engineering Specification

local GVR_G150 = {}

GVR_G150.engine = {
	idle_rpm = 725,
	redline_rpm = 4500,
	rev_limiter_rpm = 4650,
	stall_rpm = 350,
	inertia = 0.42,
	throttle_response = 1.0,
	engine_braking = 0, -- Use curve below
	-- Engine braking torque curve (negative = braking torque resisting rotation)
	engine_braking_curve = {
		{ 800, -12 },
		{ 1200, -25 },
		{ 1800, -48 },
		{ 2500, -78 },
		{ 3200, -112 },
		{ 4000, -145 },
		{ 4500, -165 },
	},
	-- Accessory drag torque (Nm) at different RPM
	accessory_drag_idle = 18,
	accessory_drag_2000 = 34,
	accessory_drag_4000 = 60,
	-- Idle governor max torque (ISC cap)
	idle_governor_max_torque = 185,
	-- Full-load torque curve (RPM, Nm) - GVR D30T 3.0L Turbo-Diesel V6
	torque_curve = {
		{ 600, 80 },
		{ 725, 145 },
		{ 900, 235 },
		{ 1100, 335 },
		{ 1300, 420 },
		{ 1500, 465 },
		{ 1700, 484 },
		{ 2000, 484 },
		{ 2500, 484 },
		{ 3000, 458 },
		{ 3500, 438 },
		{ 3750, 438 },
		{ 4000, 407 },
		{ 4250, 350 },
		{ 4500, 285 },
		{ 4650, 0 }, -- fuel cut
	},
}

GVR_G150.transmission = {
	mode = "Automatic",
	gear_ratios = { 4.696, 2.985, 2.146, 1.769, 1.520, 1.275, 1.000, 0.854, 0.689, 0.636 },
	final_drive = 3.73,
	reverse_ratio = -4.866,
	clutch_engagement = 1.0,
}

GVR_G150.tcm = {
	enabled = true,
	upshift_rpm = { 1700, 1650, 1600, 1550, 1500, 1500, 1450, 1450, 1400, 0 },
	downshift_rpm = { 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 0 },
	throttle_shift_factor = 0.20,
	kickdown_threshold = 0.75,
	lockup_speed_threshold = 45,
	lockup_throttle_threshold = math.huge, -- Speed-gate only
	thermal_limit = 140,
	limp_gear = 2,
	min_shift_interval = 0.65,
	-- Per-gear shift duration multiplier (base * this = actual duration)
	shift_duration = {
		[1] = 1.0, [2] = 1.0, [3] = 1.0, [4] = 1.0, [5] = 1.0,
		[6] = 1.0, [7] = 1.0, [8] = 1.0, [9] = 1.0, [10] = 1.0,
	},
	-- Per-gear shift torque reduction (0 = none, 1 = full cut)
	shift_torque_reduction = {
		[1] = 0.15, [2] = 0.15, [3] = 0.15, [4] = 0.15, [5] = 0.15,
		[6] = 0.15, [7] = 0.10, [8] = 0.10, [9] = 0.10, [10] = 0.05,
	},
	unlock_converter_before_downshift = true,
	skip_shift_enabled = true,
	skip_shift_load_threshold = 0.1,
}

GVR_G150.transfer_case = {
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
}

GVR_G150.efficiency_open = 0.80  -- converter open: 72-88% (midpoint)
GVR_G150.efficiency_locked = 0.90  -- converter locked: 88-92% (midpoint)
GVR_G150.efficiency_manual = 0.88  -- manual transmission

GVR_G150.differential = { mode = "Open", bias = 0.5 } -- Legacy single diff (not used with transfer case)
GVR_G150.front_differential = { mode = "Open", bias = 0.5 }
GVR_G150.rear_differential = { mode = "Open", bias = 0.5 }

GVR_G150.turbo = {
	type = "Turbo",
	max_boost_psi = 23.9, -- 1.65 bar gauge
	spool_time = 0.65, -- from idle
	wastegate_open_psi = 22.0,
	wastegate_duty_cycle = 0.5,
	bov_threshold = 0.1,
	bov_vent_rate = 2.0,
	intercooler_efficiency = 0.7,
	intercooler_pressure_loss = 0.02,
	compressor_efficiency = 0.72,
	anti_lag_enabled = false,
	anti_lag_spool_rate = 0,
	anti_lag_exhaust_temp_rise = 0,
	boost_leak_rate = 0,
	boost_leak_pressure_loss = 0,
	exhaust_energy_coeff = 1.2e-7,
	torque_per_psi = 0.015,
	reference_charge_temp = 40,
}

GVR_G150.engine_breathing = {
	ve_curve = {
		{ 725, 0.65 }, { 1000, 0.72 }, { 1500, 0.82 }, { 2000, 0.88 },
		{ 2500, 0.90 }, { 3000, 0.88 }, { 3500, 0.85 }, { 4000, 0.80 },
		{ 4500, 0.72 },
	},
	exhaust_backpressure_coeff = 2.5e-6,
	exhaust_pressure_penalty = 0.15,
	intake_temp_offset = 20,
	intercooler_effectiveness = 0.7,
	altitude_decay_per_km = 0.11,
	base_air_density = 1.225,
	alternator_coeff = 0.008,
	alternator_max_torque = 25,
	ac_enabled = true,
	ac_compressor_torque = 15,
}

GVR_G150.engine_thermal = {
	thermal = { mass = 15, coolant_capacity = 13, oil_capacity = 7.5 },
	cooling = { radiator_capacity = 13, fan_max_flow = 50, fan_on_temp = 95, fan_off_temp = 90 },
	lubrication = { oil_pressure_target = 400, oil_cooler_efficiency = 0.3 },
	damage = { overheat_threshold = 120, overheat_rate = 0.001 },
	stress = { rpm_threshold = 4500, load_threshold = 0.8 },
}

GVR_G150.brakes = {
	disc_mass = { front = 8, rear = 6 },
	pad_friction_nominal = 0.45,
	pad_fade_temp = 400,
	pad_recovery_temp = 200,
	thermal_mass = { front = 10, rear = 8 },
	cooling_rate = { front = 0.8, rear = 0.6 },
	airflow_cooling_coeff = 0.15,
}

GVR_G150.suspension = {
	front = { spring_rate = 42, motion_ratio = 0.78, travel = 0.22 },
	rear = { spring_rate = 48, motion_ratio = 0.92, travel = 0.25 },
	anti_roll_bar_rate = { front = 34000, rear = 18000 },
	damper = { compression_lowspeed = { front = 2600, rear = 2900 }, rebound_lowspeed = { front = 4300, rear = 4700 } },
	bump_stop_rate = 15000,
}

GVR_G150.tire = {
	radius_unloaded = 0.407,
	radius_loaded = 0.397,
	mass = 22,
	wheel_mass = 16,
	pressure_cold_unloaded = 240,
	pressure_cold_loaded = 275,
	compound = "Offroad",
	pacejka = {
		longitudinal_b = 10.0, longitudinal_c = 1.65, longitudinal_d = 1.0, longitudinal_e = 0.97,
		lateral_b = 10.0, lateral_c = 1.9, lateral_d = 1.0, lateral_e = 0.97,
	},
}

GVR_G150.aero = {
	Cd = 0.43,
	frontal_area = 3.21,
	Cl = 0.17,
	Cs = 0.75,
	Cn = 0.12, -- yaw moment coefficient
	wheelbase = 3.69,
}

GVR_G150.steering = {
	max_steering_angle = math.rad(37),
	steering_ratio = 16.0,
	wheelbase = 3.69,
	track_width = 1.725,
}

GVR_G150.mass = 2145, -- curb mass
GVR_G150.test_mass = 2250, -- with driver and light cargo
GVR_G150.cg_height = 0.755,
GVR_G150.cg_behind_front_axle = 1.62,
GVR_G150.inertia = { Ixx = 1050, Iyy = 4850, Izz = 5350 },

return GVR_G150