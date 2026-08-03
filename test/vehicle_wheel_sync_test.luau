local Vehicle = require("../src/physics/vehicle")

local function assertEqual(actual, expected, message)
	assert(actual == expected, string.format("%s: expected %s, got %s", message, tostring(expected), tostring(actual)))
end

local drivetrain = { wheel_speed = 0 }
local wheels = {
	{ angular_velocity = 0 },
	{ angular_velocity = 0 },
	{ angular_velocity = 0 },
	{ angular_velocity = 0 },
}

for _ = 1, 600 do
	Vehicle.syncDrivetrainWheelSpeed(drivetrain, wheels, 0, true)
	assertEqual(drivetrain.wheel_speed, 0, "stationary no-throttle wheels must not run away")
end

wheels[3].angular_velocity = 12
wheels[4].angular_velocity = 8
Vehicle.syncDrivetrainWheelSpeed(drivetrain, wheels, 0, true)
assertEqual(drivetrain.wheel_speed, 10, "RWD speed must use the driven-wheel average")

wheels[1].angular_velocity = 20
wheels[2].angular_velocity = 10
Vehicle.syncDrivetrainWheelSpeed(drivetrain, wheels, 1, true)
assertEqual(drivetrain.wheel_speed, 15, "FWD speed must use the driven-wheel average")

print("PASS vehicle wheel synchronization regression")
