local Aerodynamics = require("../src/physics/aerodynamics")

local function makeWorld(cl, wingCl)
	local world = {
		nodes = {
			{ x = 0, y = 0.5, z = 0, vx = 100, vy = 0, vz = 0, mass = 1000, fixed = false },
			{ x = 1, y = 0.5, z = 0, vx = 100, vy = 0, vz = 0, mass = 1000, fixed = false },
			{ x = 0, y = 0.5, z = 1, vx = 100, vy = 0, vz = 0, mass = 1000, fixed = false },
			{ x = 1, y = 0.5, z = 1, vx = 100, vy = 0, vz = 0, mass = 1000, fixed = false },
			{ x = 0, y = 1.0, z = 0, vx = 100, vy = 0, vz = 0, mass = 1000, fixed = false },
			{ x = 1, y = 1.0, z = 0, vx = 100, vy = 0, vz = 0, mass = 1000, fixed = false },
		},
		fixedDt = 1 / 240,
		airDensity = 1.225,
		frontalArea = 3.2,
		aeroCd = 0,
		aeroCl = cl,
		aeroCs = 0,
		aeroCm = 0,
		aeroWingCl = wingCl or 0,
		aeroWingArea = if wingCl then 1 else 0,
		aeroNodeDrag = 0,
		aeroGroundRefHeight = 0.35,
		aeroGroundEffectMin = 0.15,
		aeroGroundEffectMax = 2,
		aeroPitchBrakeCoeff = 0,
		aeroPitchPowerCoeff = 0,
		vehicle_speed = 100,
		controls = { brake = 0, throttle = 0 },
	}
	function world:terrainHeight() return 0 end
	Aerodynamics.install(world)
	return world
end

-- Negative Cl is authored as downforce and must reduce vertical velocity.
do
	local world = makeWorld(-0.4)
	world:_applyDrag()
	assert(world.nodes[1].vy < 0, "negative aero Cl must apply downward force, got vy=" .. tostring(world.nodes[1].vy))
end

-- Positive Cl is lift and must increase vertical velocity.
do
	local world = makeWorld(0.4)
	world:_applyDrag()
	assert(world.nodes[1].vy > 0, "positive aero Cl must apply upward force, got vy=" .. tostring(world.nodes[1].vy))
end

-- Negative WingCl must also apply downward force.
do
	local world = makeWorld(0, -0.4)
	world:_applyDrag()
	assert(world.nodes[5].vy < 0, "negative WingCl must apply downward force, got vy=" .. tostring(world.nodes[5].vy))
end

print("aerodynamics lift-sign regression passed")
