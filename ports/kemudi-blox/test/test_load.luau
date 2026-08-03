local WheelDynamics = require("../src/vehicle/wheel_dynamics")

local config = WheelDynamics.newConfig({ wheel_mass = 20.0, tire_radius = 0.4385 })
local state = WheelDynamics.newState()
state.tire_radius = config.tire_radius
state.inertia = WheelDynamics.computeInertia(config)
assert(state.angular_velocity == 0.0)
assert(WheelDynamics.getWheelSpeed(state) == 0.0)
WheelDynamics.integrate(state, config, 10.0, 1 / 60)
assert(state.angular_velocity == 0.0)
state.drive_torque = 500.0
WheelDynamics.integrate(state, config, 10.0, 1 / 60)
assert(state.angular_velocity > 0)
local matched = WheelDynamics.newState({ angular_velocity = 100.0 })
matched.tire_radius = config.tire_radius
matched.inertia = WheelDynamics.computeInertia(config)
WheelDynamics.integrate(matched, config, 43.85, 1 / 60)
assert(matched.slip_ratio == matched.slip_ratio)
print("All wheel dynamics load tests passed")
