local ContactModel = require("../src/physics/contact_model")

local function makeConfig(overrides)
	local values = { contact_stiffness=80000, contact_damping=5000, body_contact_stiffness=200000, body_contact_damping=15000, terrain_compliance=0.5, friction_coefficient=0.8, rolling_resistance_coeff=0.015, tire_contact_patch_area=0.03 }
	for key, value in pairs(overrides or {}) do values[key] = value end
	return ContactModel.newConfig(values)
end

-- Test newConfig
local cfg = makeConfig()
assert(cfg.contact_stiffness == 80000.0)
assert(cfg.contact_damping == 5000.0)
assert(cfg.body_contact_stiffness == 200000.0)
assert(cfg.body_contact_damping == 15000.0)
assert(cfg.terrain_compliance == 0.5)
assert(cfg.friction_coefficient == 0.8)
assert(cfg.rolling_resistance_coeff == 0.015)
assert(cfg.tire_contact_patch_area == 0.03)

-- Test overrides
local cfg2 = makeConfig({ contact_stiffness = 100000 })
assert(cfg2.contact_stiffness == 100000)

-- Test NaN clamp
local cfg3 = makeConfig({ contact_stiffness = 0/0 })
assert(cfg3.contact_stiffness == 0)

local cfg4 = makeConfig({ contact_stiffness = -100 })
assert(cfg4.contact_stiffness == 0)

local cfg5 = makeConfig({ contact_stiffness = 2e9 })
assert(cfg5.contact_stiffness == 1e9)

-- Test newState
local state = ContactModel.newState()
assert(state.total_contact_energy_lost == 0)

-- Test applyBodyFriction
local n = { vx = 10, vy = 0, vz = 5, mass = 1000 }
local normal = { 0, 1, 0 }
ContactModel.applyBodyFriction(n, normal, 0.8, 1/60)
-- Velocity should be reduced
assert(n.vx < 10)

-- Test computeRollingResistance
local f = ContactModel.computeRollingResistance(10, 5000, cfg)
assert(f == 0.015 * 5000)

local f2 = ContactModel.computeRollingResistance(0.05, 5000, cfg)
assert(f2 == 0)

local f3 = ContactModel.computeRollingResistance(10, 0, cfg)
assert(f3 == 0)

print("All contact_model tests passed")