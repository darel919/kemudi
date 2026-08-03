-- Regression for low-speed vehicle drift after the driver has stopped.
-- Run: lune run test/stop_drift_regression_test.luau

local Integration = require("../src/physics/integration")
local Vehicle = require("../src/physics/vehicle")
local Math = require("../src/math")
local Suspension = require("../src/vehicle/suspension")
local Tires = require("../src/vehicle/tires")
local Drivetrain = require("../src/vehicle/drivetrain")
local Angular = require("../src/physics/angular")

local WorldMethods = {}
Integration.install(WorldMethods, Math)
Vehicle.install(WorldMethods, Suspension, Tires, Drivetrain, nil, Math, nil)
Angular.install(WorldMethods, Math)

local function makeWorld()
	local suspension = Suspension.newConfig({
		wheels = {},
		anti_roll_bar_stiffness = 0,
		bump_stop_rate = 0,
	})
	for _ = 1, 4 do
		table.insert(suspension.wheels, Suspension.newWheelConfig({
			rest_length = 0.3,
			travel = 0.2,
			spring_rate = 30000,
			damping = 4000,
			rebound_damping = 3000,
			tire_radius = 0.3,
		}))
	end

	local world = {
		nodes = {},
		baseNodeMasses = {},
		fixedDt = 1 / 60,
		gravity = -9.81,
		airDensity = 1.225,
		dragCoefficient = 0.5,
		frontalArea = 2.2,
		steering_config = Suspension.newSteeringConfig(),
		steering_angle = 0,
		steering_states = { 0, 0 },
		suspension_config = suspension,
		suspension_states = {},
		tires = {},
		tire_thermal = Tires.newThermalParams(),
		tire_params = Tires.newPacejkaParams(),
		driveLayout = 0,
		vehicle_mass = 1000,
		previousHeading = 0,
		yawRate = 0,
		angularVelocity = { 0, 1.5, 0 },
		centerOfMass = { 0, 0, 0 },
		inertia = { 1000, 1000, 1000 },
		restPositions = {},
		restRelativePositions = {},
		shapeMatchingStiffness = 0,
		controls = { steering = 0, throttle = 0, brake = 0, clutch = 0, handbrake = false, engine_on = true },
	}
	function world:terrainHeight(_x, _z) return 0 end
	function world:terrainNormal(_x, _z) return { 0, 1, 0 } end
	function world:surfacePropertiesAt(_x, _z) return 0.94, 0.65, 0, 1, 0 end

	for _ = 1, 4 do
		table.insert(world.suspension_states, Suspension.newWheelState())
		table.insert(world.tires, Tires.newState("Street"))
	end
	for index, position in {
		{ 1.25, 0.5, -0.8 },
		{ 1.25, 0.5, 0.8 },
		{ -1.25, 0.5, -0.8 },
		{ -1.25, 0.5, 0.8 },
	} do
		table.insert(world.nodes, {
			x = position[1], y = position[2], z = position[3],
			vx = 0.1 / 3.6, vy = 0, vz = 0,
			mass = 250, invMass = 1 / 250, fixed = false,
			fx = 0, fy = 0, fz = 0, lastForce = 0,
		})
		world.nodes[index].id = index - 1
		table.insert(world.baseNodeMasses, 250)
	end
	world.drivetrain = Drivetrain.new({
		engine = {
			idle_rpm = 700,
			redline_rpm = 6500,
			rev_limiter_rpm = 6800,
			throttle_response = 0.8,
			engine_braking = 30,
			inertia = 0.32,
			torque_curve = { { 700, 100 }, { 4000, 280 }, { 6500, 180 } },
		},
		transmission = {
			mode = "Automatic",
			gear_ratios = { 3.5, 2.1, 1.4 },
			final_drive = 3.7,
			reverse_ratio = -3.2,
			clutch_engagement = 1,
		},
	})
	world.drivetrain.wheel_radius = 0.3
	return setmetatable(world, { __index = WorldMethods })
end

local world = makeWorld()
local initialX = 0
for _, node in world.nodes do initialX += node.x end

for _ = 1, 120 do
	WorldMethods._updateVehicleSystems(world)
	assert(world.angularVelocity[2] == 0, "a parked vehicle must not retain yaw angular velocity")
	WorldMethods._applyForces(world)
	WorldMethods._computeAngularVelocity(world)
	WorldMethods._applyAngularMotion(world)
	WorldMethods._integrateVelocities(world, world.fixedDt)
	WorldMethods._enforceParkedRestState(world)
end

local finalX = 0
local maxSpeed = 0
for _, node in world.nodes do
	finalX += node.x
	maxSpeed = math.max(maxSpeed, math.sqrt(node.vx * node.vx + node.vz * node.vz))
end

assert(math.abs(finalX - initialX) < 1e-4, "a parked vehicle must not drift after reaching a stop")
assert(maxSpeed < 1e-4, "a parked vehicle must settle all low-speed node velocity")
assert(world.yawRate == 0, "a parked vehicle must report zero yaw rate")
print("stop drift regression passed")
