local Collision = require("../src/physics/collision")
local Contract = require("../src/contract")
local Math = require("../src/math")

local world = {
	nodes = {},
	collisionBoxes = {},
	collisionSpheres = {},
	boundaries = {},
	telemetry = table.create(Contract.TELEMETRY_LEN, 0),
	fixedDt = 1 / 60,
	gravity = -9.81,
}

Collision.install(world, Contract, Math)

-- A wall rotated 90 degrees around Y. Its local X axis points along world +Z,
-- so the node at world +Z is inside the wall and must be pushed back along +Z.
world.collisionBoxes[1] = {
	center = { 0, 0, 0 },
	halfExtents = { 0.25, 2, 3 },
	basis = {
		{ 0, 0, 1 },
		{ 0, 1, 0 },
		{ -1, 0, 0 },
	},
	restitution = 0,
	friction = 0,
}
world.nodes[1] = {
	x = 0,
	y = 0,
	z = 0.1,
	vx = 0,
	vy = 0,
	vz = -4,
	mass = 10,
	fixed = false,
	collision = true,
}

world:_collideWithStaticGeometry()

assert(world.nodes[1].z < 0.5, "rotated static wall must correct penetration")
assert(world.nodes[1].vz >= 0, "rotated static wall must remove velocity into the wall")
print("static collision regression passed")

-- The production asset currently has four wheel/suspension nodes. Those nodes
-- skip analytic terrain collision, but must still collide with static walls.
local vehicleWorld = {
	nodes = {},
	collisionBoxes = world.collisionBoxes,
	collisionSpheres = {},
	boundaries = {},
	telemetry = table.create(Contract.TELEMETRY_LEN, 0),
	fixedDt = 1 / 60,
	gravity = -9.81,
}
for index = 1, 4 do
	table.insert(vehicleWorld.nodes, {
		x = 0,
		y = 0,
		z = 0.1,
		vx = 0,
		vy = 0,
		vz = if index == 1 then -40 / 3.6 else 0,
		mass = 10,
		fixed = false,
		collision = false,
		staticCollision = true,
	})
end
vehicleWorld._collideWithStaticGeometry = world._collideWithStaticGeometry
vehicleWorld._recordStaticImpact = world._recordStaticImpact
vehicleWorld._applyContactVelocity = world._applyContactVelocity
vehicleWorld:_collideWithStaticGeometry()
assert(vehicleWorld.nodes[1].collision == false, "wheel node must remain outside analytic terrain collision")
assert(vehicleWorld.nodes[1].staticCollision == true, "wheel node must be eligible for static geometry")
assert(vehicleWorld.nodes[1].vz <= 0.1, "four-node vehicle must not rebound from a static wall")
print("four-node vehicle static collision regression passed")

-- Static contact must feed the damage and beam-failure path, not only reverse
-- the node velocity. The front node is inside the wall and connected to a
-- rear node outside it, so the contact impulse creates beam tension.
local Constraints = require("../src/physics/constraints")
local Telemetry = require("../src/physics/telemetry")
local impactWorld = {
	nodes = {
		{
			x = 0, y = 0, z = 0.1, vx = 0, vy = 0, vz = -40 / 3.6,
			mass = 10, fixed = false, collision = false, staticCollision = true,
			invMass = 0.1,
		},
		{
			x = 0, y = 0, z = -1, vx = 0, vy = 0, vz = 0,
			mass = 10, fixed = false, collision = false, staticCollision = true,
			invMass = 0.1,
		},
	},
	beams = {
		{
			nodeA = 0, nodeB = 1, stiffness = 1000, damping = 0,
			strength = 100, yieldStrength = 55, plasticity = 0.25,
			length = 1.1, initialLength = 1.1, broken = false,
			lambda = 0, stress = 0,
		},
	},
	triangles = {},
	collisionBoxes = {},
	collisionSpheres = {},
	boundaries = {},
	telemetry = table.create(Contract.TELEMETRY_LEN, 0),
	fixedDt = 1 / 60,
	gravity = -9.81,
	restPositions = {},
	restCentroid = { 0, 0, 0 },
	volumeLambda = 0,
	volumeCompliance = 1e-5,
	bodyAttachmentSoftness = 0.35,
	rutDepthAt = function() return 0 end,
}
impactWorld.collisionBoxes[1] = {
	center = { 0, 0, 0 },
	halfExtents = { 0.25, 2, 0.25 },
	restitution = 0.0,
	friction = 0,
}
Collision.install(impactWorld, Contract, Math)
Constraints.install(impactWorld, Math)
Telemetry.install(impactWorld, Contract)
impactWorld:_collideWithStaticGeometry()
impactWorld:_updateTelemetry()
impactWorld:_updateBeamMaterialState(impactWorld.fixedDt)
assert(impactWorld.nodes[1].vz <= 0.1, "40 km/h wall impact must not rebound like an elastic collision")
assert(impactWorld.telemetry[Contract.T.DAMAGE + 1] > 0, "wall impact must accumulate damage")
assert(impactWorld.telemetry[Contract.T.IMPACT_SEVERITY + 1] > 0, "wall impact must record severity")
assert(impactWorld.beams[1].broken, "high-energy wall impact must break the loaded beam")
print("static impact damage and beam break regression passed")
