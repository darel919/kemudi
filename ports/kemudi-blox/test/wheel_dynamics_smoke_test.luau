local Loader = require("./loader")
local WheelDynamics = Loader.load("vehicle/wheel_dynamics")

local function config(overrides)
    local c = { wheel_mass = 20.0, tire_radius = 0.4385 }
    for key, value in pairs(overrides or {}) do c[key] = value end
    return WheelDynamics.newConfig(c)
end

local s = WheelDynamics.newState()
assert(s.angular_velocity == 0.0)
assert(s.inertia == 0.0)
assert(s.tire_radius == 0.0)
assert(s.is_locked == false)

local c = config()
assert(c.wheel_mass == 20.0)
assert(c.tire_radius == 0.4385)
assert(WheelDynamics.computeInertia(c) > 0)
s.tire_radius = c.tire_radius
s.inertia = WheelDynamics.computeInertia(c)
assert(WheelDynamics.getSlipRatio(s) == 0.0)
assert(WheelDynamics.getWheelSpeed(s) == 0.0)

local tel = WheelDynamics.telemetry(s)
assert(tel.angular_velocity == 0.0)
assert(tel.slip_ratio == 0.0)
assert(tel.angular_energy == 0.0)
assert(tel.is_locked == false)

WheelDynamics.integrate(s, c, 10.0, 1/60)
assert(s.angular_velocity == 0.0)
s.drive_torque = 500.0
WheelDynamics.integrate(s, c, 10.0, 1/60)
assert(s.angular_velocity > 0.0)
assert(s.angular_energy > 0.0)

local s2 = WheelDynamics.newState({ is_locked = true })
s2.tire_radius = c.tire_radius
s2.inertia = WheelDynamics.computeInertia(c)
WheelDynamics.integrate(s2, c, 10.0, 1/60)
assert(s2.angular_velocity == 0.0)

local s3 = WheelDynamics.newState({ angular_velocity = 0/0 })
s3.tire_radius = c.tire_radius
s3.inertia = WheelDynamics.computeInertia(c)
WheelDynamics.integrate(s3, c, 10.0, 1/60)
assert(s3.angular_velocity == 0.0)

print("All wheel_dynamics tests passed")
