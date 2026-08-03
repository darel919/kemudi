local Angular = require("../src/physics/angular")

local World = {}
Angular.install(World, {})

World.nodes = {
	{ x = 1, y = 0, z = 0, vx = 0, vy = 0, vz = 0, fixed = false },
}
World.centerOfMass = { 0, 0, 0 }
World.angularVelocity = { 0, 0, 1 }
World.fixedDt = 1 / 240

World:_applyAngularMotion()
local expectedVy = World.nodes[1].vy
World:_applyAngularMotion()

assert(math.abs(expectedVy - 1) < 1e-9, "initial tangential velocity should equal omega cross r")
assert(
	math.abs(World.nodes[1].vy - expectedVy) < 1e-9,
	"reapplying the same angular velocity must not accumulate omega cross r"
)

print("angular motion injection regression passed")
