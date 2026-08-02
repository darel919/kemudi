-- Lune compatibility and production behavior tests for the actual vehicle modules.
-- Run: lune run test/module_compat_test.luau

local tires = require("../src/vehicle/tires")
local drivetrain = require("../src/vehicle/drivetrain")
local suspension = require("../src/vehicle/suspension")
local safety = require("../src/vehicle/safety")

local function approx(actual, expected, tolerance, message)
	assert(math.abs(actual - expected) <= tolerance,
		message or string.format("expected %.8f, got %.8f", expected, actual))
end

assert(type(tires.newState) == "function")
assert(type(drivetrain.new) == "function")
assert(type(suspension.newConfig) == "function")
assert(type(safety.newState) == "function")

-- Tire compound, thermal, pressure, wear, and failure state.
local sport = tires.newState("Sport")
assert(sport.grip_factor == 0.95, "Sport compound must start at its compound grip")

local thermal = tires.newThermalParams({
	heat_generation_rate = 1.0,
	heat_dissipation_rate = 0.0,
	thermal_mass = 100.0,
})
tires.update(sport, thermal, 0.2, 0.05, 3000.0, 0.8, 20.0, 1.0)
assert(sport.temperature > 20.0, "slip must generate heat")
assert(sport.pressure > sport.nominal_pressure, "temperature must increase pressure")
assert(sport.wear > 0.0, "slip must generate wear")
assert(sport.center_wear > 0.0, "combined slip must produce center wear")
assert(sport.shoulder_wear > 0.0, "combined slip must produce shoulder wear")

local pacejka = tires.calculateForces(0.1, 0.05, 3000.0, sport.grip_factor)
assert(pacejka.longitudinal > 0.0, "positive slip must produce drive force")
assert(pacejka.lateral < 0.0, "positive slip angle must produce lateral force")
assert(pacejka.combined_magnitude <= pacejka.combined_limit + 1e-6,
	"combined tire force must stay within its limit")

local overheated = tires.newState("Street")
local aggressive_thermal = tires.newThermalParams({
	heat_generation_rate = 1.0,
	heat_dissipation_rate = 0.0,
	thermal_mass = 1.0,
})
tires.update(overheated, aggressive_thermal, 0.5, 0.0, 4000.0, 0.8, 20.0, 0.3)
assert(overheated.has_blister, "extreme temperature must blister the tire")
assert(overheated.grip_factor < 0.5, "blistered tire must lose grip")

local worn = tires.newState("Street")
worn.wear = 0.96
tires.update(worn, tires.newThermalParams(), 0.0, 0.0, 0.0, 0.0, 20.0, 1 / 60)
assert(worn.is_flat, "near-total wear must flatten the tire")

-- Engine/transmission torque propagation.
local engine = drivetrain.newEngine()
approx(drivetrain.engineTorqueAtRPM(engine, 4000.0), 280.0, 1e-6)
local transmission = drivetrain.newTransmission()
local gear1_ratio = 3.5 * 3.7
approx(drivetrain.totalRatio(transmission), gear1_ratio, 1e-6)
approx(drivetrain.wheelTorque(transmission, 280.0), 280.0 * gear1_ratio, 1e-6)

local rpm_from_one_rad = drivetrain.engineRPMFromWheelSpeed(transmission, 1.0)
approx(rpm_from_one_rad, gear1_ratio * 60.0 / (2 * math.pi), 1e-6)
assert(drivetrain.shiftUp(transmission), "gear 1 must shift up")
assert(transmission.current_gear == 2, "shift up must select gear 2")
assert(drivetrain.shiftDown(transmission), "gear 2 must shift down")
assert(transmission.current_gear == 1, "shift down must select gear 1")

local differential = drivetrain.newDifferential({ mode = "Open" })
local left_open, right_open = drivetrain.applyDifferential(differential, 1000.0, 0.2, 1.0, 0.0, 0.0)
approx(left_open, 500.0, 1e-6)
approx(right_open, 500.0, 1e-6)

local limited = drivetrain.newDifferential({ mode = "LimitedSlip", bias = 1.0 })
local left_lsd, right_lsd = drivetrain.applyDifferential(limited, 1000.0, 0.2, 1.0, 0.0, 0.0)
assert(left_lsd < right_lsd, "limited-slip differential must bias torque toward the higher-grip side")
approx(left_lsd + right_lsd, 1000.0, 1e-6, "differential must conserve torque")

local shock = drivetrain.newClutchShock()
drivetrain.applyClutchShock(shock, 1500.0, 1.0, 1 / 60)
assert(shock.magnitude > 0.0, "RPM mismatch must create clutch shock")
assert(shock.torque_spike > 0.0, "RPM mismatch must create torque spike")

-- Suspension and safety production APIs.
local suspension_config = suspension.newConfig()
assert(#suspension_config.wheels == 4, "default suspension must have four wheels")
local wheel_result, compression, airborne = suspension.raycastWheel(
	suspension_config.wheels[1], { 0.0, 0.5, 0.0 }, 0.0, 0.0)
assert(not airborne, "wheel at ground contact must not be airborne")
assert(wheel_result.force > 0.0, "compressed wheel must produce spring force")
assert(compression > 0.0, "compressed wheel must report compression")

local abs = safety.newABS()
local pressure = safety.updateABS(abs, { 5.0, 28.0, 28.0, 28.0 }, 30.0, 0.8, 1 / 60)
assert(pressure[1] < pressure[2], "ABS must release pressure on the slipping wheel")

print("production vehicle modules and behavior tests passed")
