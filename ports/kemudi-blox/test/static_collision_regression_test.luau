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
		vz = if index == 1 then -4 else 0,
		mass = 10,
		fixed = false,
		collision = false,
		staticCollision = true,
	})
end
vehicleWorld._collideWithStaticGeometry = world._collideWithStaticGeometry
vehicleWorld._applyContactVelocity = world._applyContactVelocity
vehicleWorld:_collideWithStaticGeometry()
assert(vehicleWorld.nodes[1].collision == false, "wheel node must remain outside analytic terrain collision")
assert(vehicleWorld.nodes[1].staticCollision == true, "wheel node must be eligible for static geometry")
assert(vehicleWorld.nodes[1].vz >= 0, "four-node vehicle must stop at a static wall")
print("four-node vehicle static collision regression passed")
