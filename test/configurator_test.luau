--[[ Configurator unit tests for Lune.
	Validates manifest building, validation, configurator apply, and presets.
	Run: lune run test/configurator_test.luau
--]]

local process = require("@lune/process")
local Manifest = require("../src/configurator/manifest")
local Configurator = require("../src/configurator/vehicle_configurator")
local Presets = require("../src/configurator/param_presets")

local passed = 0
local failed = 0
local total = 0

local function test(name, fn)
	total += 1
	local ok, err = pcall(fn)
	if ok then
		passed += 1
		print(string.format("  PASS  %s", name))
	else
		failed += 1
		print(string.format("  FAIL  %s: %s", name, tostring(err)))
	end
end

local function check(cond, msg)
	if not cond then error(msg or "assertion failed") end
end

local function approx(a, b, tol)
	tol = tol or 0.01
	if math.abs(a - b) > tol then
		error(string.format("expected %.4f, got %.4f (diff %.4f)", b, a, math.abs(a - b)))
	end
end

-- ============================================================
-- Mock vehicle asset definition (matches premium_sportscar)
-- ============================================================

local function makeSportscarDefinition()
	return {
		engineConfig = {
			idle_rpm = 850,
			redline_rpm = 8000,
			rev_limiter_rpm = 8200,
			throttle_response = 1.1,
			engine_braking = 0.25,
			torque_curve = {
				{0, 150}, {1000, 300}, {2000, 420}, {3000, 500},
				{4000, 540}, {5000, 520}, {6000, 480}, {7000, 420}, {8000, 350},
			},
		},
		drivetrainConfig = {
			transmission = {
				mode = "Automatic",
				gear_ratios = {4.5, 2.4, 1.7, 1.2, 0.9, 0.7},
				final_drive = 3.4,
				reverse_ratio = -3.0,
				clutch_engagement = 0.5,
			},
		},
		suspensionConfig = {
			wheels = {
				{ spring_rate = 36000, damping = 5000, rebound_damping = 3200, travel = 0.18, tire_radius = 0.34, rest_length = 0.33, unsprung_mass = 14 },
				{ spring_rate = 36000, damping = 5000, rebound_damping = 3200, travel = 0.18, tire_radius = 0.34, rest_length = 0.33, unsprung_mass = 14 },
				{ spring_rate = 40000, damping = 5500, rebound_damping = 3500, travel = 0.18, tire_radius = 0.34, rest_length = 0.33, unsprung_mass = 14 },
				{ spring_rate = 40000, damping = 5500, rebound_damping = 3500, travel = 0.18, tire_radius = 0.34, rest_length = 0.33, unsprung_mass = 14 },
			},
			anti_roll_bar_stiffness = 2200,
		},
		simulationConfig = {
			aero = {
				air_density = 1.225,
				frontal_area = 2.05,
				cd = 0.31,
				cl = -0.18,
				cs = 0, cm = 0, wing_cl = 0, wing_area = 0,
				node_drag = 0, ground_ref_height = 0, pitch_brake_coeff = 0,
				pitch_power_coeff = 0, ground_effect_min = 0.001, ground_effect_max = 1,
			},
			brakes = {
				brake_torque_max = 3200,
				bias_front = 0.6, disc_mass = 5, disc_specific_heat = 500,
				disc_area = 0.02, convective_coeff = 75, pad_friction_nominal = 0.42,
				fluid_boil_temp = 300, fade_onset_temp = 550, fade_critical_temp = 800,
				max_disc_temp = 1000, contact_patch_radius = 0.4385,
				airflow_speed_reference = 50, fluid_fade_end_temp = 500,
			},
			turbo = {
				type = "Turbo",
				max_boost_psi = 12,
				spool_time = 1.0,
				torque_per_psi = 0.04,
				reference_charge_temp = 25,
				wastegate_open_psi = 15,
			},
			wheel_dynamics = {
				wheels = {
					{ wheel_mass = 15, tire_radius = 0.34 },
					{ wheel_mass = 15, tire_radius = 0.34 },
					{ wheel_mass = 15, tire_radius = 0.34 },
					{ wheel_mass = 15, tire_radius = 0.34 },
				},
			},
			suspension_kinematics = {
				camber_gain = 0, camber_at_rest = 0, toe_gain = 0, toe_at_rest = 0,
				roll_center_height = 0, roll_center_migration = 0, anti_dive_pct = 0,
				anti_squat_pct = 0, kingpin_inclination = 0, scrub_radius = 0,
				caster_angle = 0, max_compression = 0.18, max_extension = 0.1,
			},
			engine_breathing = {
				ve_curve = {{1000, 0.6}, {4000, 0.85}, {8000, 0.7}},
				exhaust_backpressure_coeff = 0, intake_temp_offset = 0,
				intercooler_effectiveness = 0, base_air_density = 1.225,
				altitude_decay_per_km = 0, alternator_coeff = 0,
				alternator_max_torque = 0, ac_compressor_torque = 0,
				exhaust_pressure_penalty = 0,
			},
			angular = {
				inertia = {900, 2600, 2900},
				shape_matching_stiffness = 500,
				angular_damping = 0.5,
			},
			solver = {
				volume_compliance = 0.0001,
				body_attachment_softness = 0.1,
				iterations = 8,
				max_iterations = 12,
				velocity_damping = 0.01,
			},
			environment = { ambient_temperature = 25 },
			contact = {
				contact_stiffness = 100000,
				contact_damping = 1000,
				body_contact_stiffness = 50000,
				body_contact_damping = 500,
				terrain_compliance = 0.001,
				friction_coefficient = 0.9,
				rolling_resistance_coeff = 0.015,
				tire_contact_patch_area = 0.02,
			},
			terrain = {
				surface_hardness = 1.0,
				rut_recovery_rate = 0.1,
				max_rut_depth = 0.05,
				spread_factor = 0.5,
				recovery_depth_damping = 0.8,
				resistance_max = 0.3,
			},
			collision = {
				cell_size = 2,
				grid_extent = 64,
				contact_radius = 0.5,
			},
			bushings = {},
		},
	}
end

-- Truck without turbo
local function makeTruckDefinition()
	return {
		engineConfig = {
			idle_rpm = 700,
			redline_rpm = 4500,
			rev_limiter_rpm = 4800,
			throttle_response = 0.6,
			engine_braking = 0.4,
			torque_curve = {
				{0, 200}, {700, 400}, {1500, 750}, {2500, 780}, {3500, 640}, {4500, 380},
			},
		},
		drivetrainConfig = {
			transmission = {
				mode = "Automatic",
				gear_ratios = {4.0, 2.5, 1.5, 1.0},
				final_drive = 4.1,
				reverse_ratio = -3.5,
				clutch_engagement = 0.4,
			},
		},
		suspensionConfig = {
			wheels = {
				{ spring_rate = 42000, damping = 5000, rebound_damping = 3500, travel = 0.15, tire_radius = 0.42, rest_length = 0.4, unsprung_mass = 25 },
				{ spring_rate = 42000, damping = 5000, rebound_damping = 3500, travel = 0.15, tire_radius = 0.42, rest_length = 0.4, unsprung_mass = 25 },
				{ spring_rate = 48000, damping = 6000, rebound_damping = 4000, travel = 0.15, tire_radius = 0.42, rest_length = 0.4, unsprung_mass = 28 },
				{ spring_rate = 48000, damping = 6000, rebound_damping = 4000, travel = 0.15, tire_radius = 0.42, rest_length = 0.4, unsprung_mass = 28 },
			},
			anti_roll_bar_stiffness = 3000,
		},
		simulationConfig = {
			aero = {
				air_density = 1.225,
				frontal_area = 3.6,
				cd = 0.42, cl = 0.12, cs = 0, cm = 0,
				wing_cl = 0, wing_area = 0, node_drag = 0,
				ground_ref_height = 0, pitch_brake_coeff = 0,
				pitch_power_coeff = 0, ground_effect_min = 0.001, ground_effect_max = 1,
			},
			brakes = {
				brake_torque_max = 3500,
				bias_front = 0.6, disc_mass = 8, disc_specific_heat = 500,
				disc_area = 0.03, convective_coeff = 75, pad_friction_nominal = 0.42,
				fluid_boil_temp = 300, fade_onset_temp = 550, fade_critical_temp = 800,
				max_disc_temp = 1000, contact_patch_radius = 0.42,
				airflow_speed_reference = 40, fluid_fade_end_temp = 500,
			},
			-- NO turbo section
			wheel_dynamics = {
				wheels = {
					{ wheel_mass = 25, tire_radius = 0.42 },
					{ wheel_mass = 25, tire_radius = 0.42 },
					{ wheel_mass = 28, tire_radius = 0.42 },
					{ wheel_mass = 28, tire_radius = 0.42 },
				},
			},
			suspension_kinematics = {
				camber_gain = 0, camber_at_rest = 0, toe_gain = 0, toe_at_rest = 0,
				roll_center_height = 0, roll_center_migration = 0, anti_dive_pct = 0,
				anti_squat_pct = 0, kingpin_inclination = 0, scrub_radius = 0,
				caster_angle = 0, max_compression = 0.15, max_extension = 0.08,
			},
			engine_breathing = {
				ve_curve = {{700, 0.5}, {2500, 0.75}, {4500, 0.6}},
				exhaust_backpressure_coeff = 0, intake_temp_offset = 0,
				intercooler_effectiveness = 0, base_air_density = 1.225,
				altitude_decay_per_km = 0, alternator_coeff = 0,
				alternator_max_torque = 0, ac_compressor_torque = 0,
				exhaust_pressure_penalty = 0,
			},
			angular = {
				inertia = {1800, 5500, 6000},
				shape_matching_stiffness = 400,
				angular_damping = 0.6,
			},
			solver = {
				volume_compliance = 0.0001,
				body_attachment_softness = 0.1,
				iterations = 8, max_iterations = 12,
				velocity_damping = 0.01,
			},
			environment = { ambient_temperature = 25 },
			contact = {
				contact_stiffness = 100000, contact_damping = 1000,
				body_contact_stiffness = 50000, body_contact_damping = 500,
				terrain_compliance = 0.001, friction_coefficient = 0.9,
				rolling_resistance_coeff = 0.02, tire_contact_patch_area = 0.03,
			},
			terrain = {
				surface_hardness = 1.0, rut_recovery_rate = 0.1,
				max_rut_depth = 0.05, spread_factor = 0.5,
				recovery_depth_damping = 0.8, resistance_max = 0.3,
			},
			collision = { cell_size = 2, grid_extent = 64, contact_radius = 0.5 },
			bushings = {},
		},
	}
end

-- ============================================================
-- Mock World (minimal, just enough for configurator tests)
-- ============================================================

local function makeMockWorld(definition)
	local w = {
		drivetrain = {
			engine = {
				idle_rpm = definition.engineConfig.idle_rpm,
				redline_rpm = definition.engineConfig.redline_rpm,
				rev_limiter_rpm = definition.engineConfig.rev_limiter_rpm,
				throttle_response = definition.engineConfig.throttle_response,
				engine_braking = definition.engineConfig.engine_braking,
				torque_curve = {},
			},
			transmission = {
				mode = definition.drivetrainConfig.transmission.mode,
				gear_ratios = {},
				final_drive = definition.drivetrainConfig.transmission.final_drive,
				reverse_ratio = definition.drivetrainConfig.transmission.reverse_ratio,
			},
			wheel_radius = 0.34,
			tcm = {
				torque_curve = {},
				gear_ratios = {},
				final_drive = definition.drivetrainConfig.transmission.final_drive,
				peak_torque = 540,
			},
		},
		turbo_config = nil,  -- set below
		turbo_state = nil,
		engine_thermal = {
			cooling = {
				radiator_capacity = 2.8,
				coolant_capacity = 2.0,
				fan_activation_temp = 93,
				fan_deactivation_temp = 88,
			},
			thermal = { coolant_capacity = 2.0 },
		},
		suspension_config = {
			wheels = {},
			anti_roll_bar_stiffness = 2200,
		},
		wheel_dynamics_configs = {},
		wheel_dynamics_states = {},
		brake_config = {
			brake_torque_max = definition.simulationConfig.brakes.brake_torque_max,
		},
		aeroCd = definition.simulationConfig.aero.cd,
		aeroCl = definition.simulationConfig.aero.cl,
		dragCoefficient = definition.simulationConfig.aero.cd,
	}

	-- Deep copy torque curves
	for _, pt in definition.engineConfig.torque_curve do
		table.insert(w.drivetrain.engine.torque_curve, { pt[1], pt[2] })
		table.insert(w.drivetrain.tcm.torque_curve, { pt[1], pt[2] })
	end
	for _, r in definition.drivetrainConfig.transmission.gear_ratios do
		table.insert(w.drivetrain.transmission.gear_ratios, r)
		table.insert(w.drivetrain.tcm.gear_ratios, r)
	end

	-- Suspension wheels
	for _, wh in definition.suspensionConfig.wheels do
		local wheelCopy = {}
		for k, v in wh do wheelCopy[k] = v end
		table.insert(w.suspension_config.wheels, wheelCopy)
	end
	w.suspension_config.anti_roll_bar_stiffness = definition.suspensionConfig.anti_roll_bar_stiffness

	-- Wheel dynamics
	for _, wd in definition.simulationConfig.wheel_dynamics.wheels do
		table.insert(w.wheel_dynamics_configs, { wheel_mass = wd.wheel_mass, tire_radius = wd.tire_radius })
		table.insert(w.wheel_dynamics_states, { tire_radius = wd.tire_radius, inertia = 0 })
	end

	-- Turbo (if present)
	if definition.simulationConfig.turbo then
		w.turbo_config = {}
		for k, v in definition.simulationConfig.turbo do w.turbo_config[k] = v end
		w.turbo_state = { boost_pressure = 0, spool_position = 0, is_boosting = false }
	end

	return w
end

-- ============================================================
-- MANIFEST TESTS
-- ============================================================

print("\n=== Manifest Tests ===")

test("manifest builds from sportscar definition", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	check(manifest.version == 1, "version should be 1")
	check(manifest.categories.engine ~= nil, "should have engine")
	check(manifest.categories.turbo ~= nil, "should have turbo (sportscar has turbo)")
	check(manifest.categories.cooling ~= nil, "should have cooling")
	check(manifest.categories.suspension ~= nil, "should have suspension")
	check(manifest.categories.brakes ~= nil, "should have brakes")
	check(manifest.categories.transmission ~= nil, "should have transmission")
	check(manifest.categories.aerodynamics ~= nil, "should have aerodynamics")
end)

test("manifest builds from truck definition (no turbo)", function()
	local def = makeTruckDefinition()
	local manifest = Manifest.build(def)
	check(manifest.categories.engine ~= nil, "should have engine")
	check(manifest.categories.turbo == nil, "truck has no turbo → should NOT have turbo category")
	check(manifest.categories.cooling ~= nil, "should have cooling")
	check(manifest.categories.suspension ~= nil, "should have suspension")
end)

test("engine manifest has correct ranges", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local engine = manifest.categories.engine
	check(engine.editable == true, "engine should be editable")
	check(engine.params.idle_rpm ~= nil, "should have idle_rpm")
	check(engine.params.idle_rpm.default == 850, "default idle_rpm should be 850")
	check(engine.params.redline_rpm.min < 8000, "redline min should be below 8000")
	check(engine.params.redline_rpm.max > 8000, "redline max should be above 8000")
	check(engine.params.torque_preset ~= nil, "should have torque_preset")
end)

test("suspension manifest is per-wheel", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local susp = manifest.categories.suspension
	check(susp.editable == true, "suspension should be editable")
	check(susp.per_wheel == true, "suspension should be per-wheel")
	check(susp.per_wheel_params.FL ~= nil, "should have FL params")
	check(susp.per_wheel_params.FL.spring_rate ~= nil, "FL should have spring_rate")
	check(susp.shared_params.anti_roll_bar_stiffness ~= nil, "should have shared anti-roll bar")
end)

-- ============================================================
-- VALIDATION TESTS
-- ============================================================

print("\n=== Validation Tests ===")

test("validate engine redline_rpm in range", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "engine", "redline_rpm", 9000)
	check(ok, "9000 should be in range: " .. tostring(err))
end)

test("reject engine redline_rpm below minimum", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "engine", "redline_rpm", 1000)
	check(not ok, "1000 should be below minimum")
	check(err:find("below minimum"), "error should mention below minimum: " .. tostring(err))
end)

test("reject unknown category", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "nonexistent", "param", 1)
	check(not ok, "should reject unknown category")
	check(err:find("not available"), "error should mention not available")
end)

test("reject turbo on truck", function()
	local manifest = Manifest.build(makeTruckDefinition())
	local ok, err = Manifest.validate(manifest, "turbo", "max_boost_psi", 15)
	check(not ok, "truck should not have turbo category")
	check(err:find("not available"), "error should mention not available")
end)

test("validate turbo max_boost_psi in range", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "turbo", "max_boost_psi", 18)
	check(ok, "18 psi should be in range: " .. tostring(err))
end)

test("reject turbo max_boost_psi above max", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "turbo", "max_boost_psi", 100)
	check(not ok, "100 psi should be above max")
end)

test("validate suspension per-wheel param", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "suspension", "spring_rate", 50000, "FL")
	check(ok, "50000 spring_rate should be in range: " .. tostring(err))
end)

test("reject suspension without wheelId", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "suspension", "spring_rate", 50000)
	check(not ok, "should require wheelId")
	check(err:find("wheelId"), "error should mention wheelId")
end)

test("validate transmission final_drive", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "transmission", "final_drive", 4.5)
	check(ok, "4.5 final drive should be in range: " .. tostring(err))
end)

test("validate aerodynamics cd", function()
	local manifest = Manifest.build(makeSportscarDefinition())
	local ok, err = Manifest.validate(manifest, "aerodynamics", "cd", 0.25)
	check(ok, "0.25 cd should be in range: " .. tostring(err))
end)

-- ============================================================
-- CONFIGURATOR APPLY TESTS
-- ============================================================

print("\n=== Configurator Apply Tests ===")

test("apply engine redline change", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("engine", "redline_rpm", 9000)
	check(ok, "apply should succeed: " .. tostring(err))
	approx(world.drivetrain.engine.redline_rpm, 9000)
end)

test("apply engine redline auto-updates rev_limiter", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	conf:apply("engine", "redline_rpm", 9500)
	-- rev_limiter should be at least redline + 200
	check(world.drivetrain.engine.rev_limiter_rpm >= 9500, "rev_limiter should be >= new redline")
end)

test("apply turbo boost change", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("turbo", "max_boost_psi", 15)
	check(ok, "turbo boost apply should succeed: " .. tostring(err))
	approx(world.turbo_config.max_boost_psi, 15)
end)

test("disable turbo", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("turbo", "enabled", false)
	check(ok, "turbo disable should succeed: " .. tostring(err))
	check(world.turbo_config.type == "None", "turbo type should be None")
	check(world.turbo_state.boost_pressure == 0, "turbo boost should be 0")
end)

test("remove radiator (set capacity to 0)", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("cooling", "radiator_capacity", 0)
	check(ok, "radiator removal should succeed: " .. tostring(err))
	approx(world.engine_thermal.cooling.radiator_capacity, 0)
end)

test("apply suspension spring_rate", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("suspension", "spring_rate", 55000, "FL")
	check(ok, "suspension apply should succeed: " .. tostring(err))
	approx(world.suspension_config.wheels[1].spring_rate, 55000)
end)

test("apply tire_radius syncs wheel_dynamics", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("suspension", "tire_radius", 0.40, "RL")
	check(ok, "tire_radius apply should succeed: " .. tostring(err))
	approx(world.suspension_config.wheels[3].tire_radius, 0.40)
	approx(world.wheel_dynamics_configs[3].tire_radius, 0.40)
end)

test("apply brake torque", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("brakes", "brake_torque_max", 4000)
	check(ok, "brake apply should succeed: " .. tostring(err))
	approx(world.brake_config.brake_torque_max, 4000)
end)

test("apply transmission final_drive", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("transmission", "final_drive", 4.5)
	check(ok, "transmission apply should succeed: " .. tostring(err))
	approx(world.drivetrain.transmission.final_drive, 4.5)
	approx(world.drivetrain.tcm.final_drive, 4.5)
end)

test("apply aero cd", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("aerodynamics", "cd", 0.25)
	check(ok, "aero apply should succeed: " .. tostring(err))
	approx(world.aeroCd, 0.25)
	approx(world.dragCoefficient, 0.25)
end)

test("reject invalid value", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local ok, err = conf:apply("engine", "redline_rpm", 100)
	check(not ok, "should reject out-of-range value")
end)

-- ============================================================
-- PRESETS TESTS
-- ============================================================

print("\n=== Presets Tests ===")

test("torque curve sport preset scales values", function()
	local base = { {1000, 300}, {4000, 500}, {7000, 400} }
	local sport = Presets.getTorqueCurve("sport", base)
	check(sport ~= nil, "sport preset should return a curve")
	check(#sport == 3, "should have same number of points")
	-- Sport should scale up
	check(sport[2][2] > base[2][2], "sport torque should be higher than stock")
end)

test("torque curve stock preset returns original", function()
	local base = { {1000, 300}, {4000, 500} }
	local stock = Presets.getTorqueCurve("stock", base)
	check(stock == base, "stock should return the same reference")
end)

test("gear ratio short preset adjusts ratios", function()
	local ratios = { 4.5, 2.4, 1.7, 1.2 }
	local short, finalDrive = Presets.getGearPreset("short", ratios, 3.4)
	check(short ~= nil, "short preset should return ratios")
	check(short[1] > ratios[1], "short should have higher ratios (better accel)")
	check(finalDrive < 3.4, "short final drive should be lower")
end)

test("gear ratio tall preset adjusts ratios", function()
	local ratios = { 4.5, 2.4, 1.7, 1.2 }
	local tall, finalDrive = Presets.getGearPreset("tall", ratios, 3.4)
	check(tall[1] < ratios[1], "tall should have lower ratios (higher top speed)")
	check(finalDrive > 3.4, "tall final drive should be higher")
end)

test("tire compound returns correct modifiers", function()
	local perf = Presets.getTireCompound("performance")
	check(perf ~= nil, "performance compound should exist")
	check(perf.grip_factor > 1.0, "performance should have higher grip")
	local eco = Presets.getTireCompound("economy")
	check(eco.grip_factor < 1.0, "economy should have lower grip")
end)

-- ============================================================
-- SNAPSHOT TESTS
-- ============================================================

print("\n=== Snapshot Tests ===")

test("snapshot captures current values", function()
	local def = makeSportscarDefinition()
	local manifest = Manifest.build(def)
	local world = makeMockWorld(def)
	local conf = Configurator.new(manifest, world)

	local snap = conf:snapshot()
	check(snap.engine ~= nil, "snapshot should have engine")
	check(snap.engine.redline_rpm == 8000, "engine redline should be 8000")
	check(snap.turbo ~= nil, "snapshot should have turbo")
	approx(snap.turbo.max_boost_psi, 12)
	check(snap.transmission ~= nil, "snapshot should have transmission")
	approx(snap.transmission.final_drive, 3.4)
end)

-- ============================================================
-- Summary
-- ============================================================

print(string.format("\n=== Results: %d/%d passed, %d failed ===", passed, total, failed))
if failed > 0 then process.exit(1) end
