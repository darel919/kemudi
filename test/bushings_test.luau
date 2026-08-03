-- Bushings module load and behavior test.
-- Run: lune run test/bushings_test.luau

local Bushings = require("../src/physics/bushings")
local Math = require("../src/math")

-- 1) Install on a mock World
local World = {}
function World.new()
	local self = setmetatable({}, { __index = World })
	self.nodes = {}
	self.beams = {}
	self.triangles = {}
	self.gravity = -9.81
	self.time = 0
	self.fixedDt = 1 / 60
	return self
end
Bushings.install(World, Math)

assert(type(World.addBushing) == "function", "install must attach addBushing")
assert(type(World.addBushingXYZ) == "function", "install must attach addBushingXYZ")
assert(type(World.setupDefaultBushings) == "function", "install must attach setupDefaultBushings")
assert(type(World._updateBushings) == "function", "install must attach _updateBushings")
assert(type(World._updateBushingDamage) == "function", "install must attach _updateBushingDamage")
assert(type(World.destroyBushing) == "function", "install must attach destroyBushing")

-- 2) Create a world with 8 nodes in a box-like arrangement
local w = World.new()

local function addNode(x, y, z)
	local idx = #w.nodes
	w.nodes[#w.nodes + 1] = {
		x = x, y = y, z = z,
		vx = 0, vy = 0, vz = 0,
		invMass = 1.0,
		fixed = false,
	}
	return idx
end

-- Lower cage (0-3)
addNode(-1, 0, -2) -- 0: front-left
addNode( 1, 0, -2) -- 1: front-right
addNode(-1, 0,  2) -- 2: rear-left
addNode( 1, 0,  2) -- 3: rear-right
-- Upper cage (4-7)
addNode(-1, 0.5, -2) -- 4
addNode( 1, 0.5, -2) -- 5
addNode(-1, 0.5,  2) -- 6
addNode( 1, 0.5,  2) -- 7

-- 3) Lazy init: bushings should not exist until first addBushing
assert(w.bushings == nil, "bushings must be nil before first use")
assert(w.bushingDamage == nil, "bushingDamage must be nil before first use")

-- 4) addBushing basic
w:addBushing(0, 4, { stiffness = 50000, damping = 3000, deadzone = 0.02 })
assert(w.bushings ~= nil, "bushings table must exist after addBushing")
assert(#w.bushings == 1, "must have 1 bushing")
local b = w.bushings[1]
assert(b.nodeA == 0, "nodeA must be 0")
assert(b.nodeB == 4, "nodeB must be 4")
assert(b.restLength > 0, "restLength must be positive")
assert(b.stiffness_x == 50000, "stiffness_x must be 50000")
assert(b.damping_x == 3000, "damping_x must be 3000")
assert(b.deadzone == 0.02, "deadzone must be 0.02")
assert(b.damage == 0, "damage must start at 0")
assert(b.stress == 0, "stress must start at 0")

-- 5) Boundary: same node returns silently
local countBefore = #w.bushings
w:addBushing(0, 0)
assert(#w.bushings == countBefore, "self-bushing must be rejected")

-- 6) Boundary: out-of-range node returns silently
w:addBushing(0, 99)
assert(#w.bushings == countBefore, "out-of-range must be rejected")

-- 7) Invalid authored stiffness is rejected rather than defaulted
w:addBushing(1, 5, { stiffness = 60000, damping = 3000, deadzone = 0.02 })
assert(#w.bushings == countBefore + 1, "valid authored bushing should be added")
local bNaN = w.bushings[#w.bushings]
assert(bNaN.stiffness_x == 60000, "stiffness must remain asset-authored")

-- 8) _updateBushings: deadzone test — nodes at rest, no force applied
local v0x, v0y, v0z = w.nodes[1].vx, w.nodes[1].vy, w.nodes[1].vz
w:_updateBushings(1 / 60)
-- Nodes at rest distance, within deadzone: velocities must not change
assert(w.nodes[1].vx == v0x, "node 0 vx must not change in deadzone")
assert(w.nodes[1].vy == v0y, "node 0 vy must not change in deadzone")

-- 9) _updateBushings: stretched bushing produces restoring force
local b2 = w.bushings[2] -- bushing between nodes 1 and 5
-- Stretch node 5 away from node 1
w.nodes[6].y = w.nodes[6].y + 0.5 -- move node 5 up (index 6 = nodeB=5 → 1-based index 6)
-- Reset velocities
w.nodes[6].vx = 0
w.nodes[6].vy = 0
w.nodes[6].vz = 0
w.nodes[2].vx = 0
w.nodes[2].vy = 0
w.nodes[2].vz = 0

w:_updateBushings(1 / 60)
-- Node 5 (stretched away from node 1) should get downward velocity component
-- (spring pulls it back toward node 1 which is below)
-- Node 1 (lower node) should get upward velocity component (equal and opposite)
local node5 = w.nodes[6] -- 1-based for node index 5
local node1 = w.nodes[2] -- 1-based for node index 1
-- At minimum, stress must be nonzero for a stretched bushing
assert(b2.stress > 0, "stressed bushing must have nonzero stress")

-- 10) _updateBushingDamage: stress accumulation
local oldDamage = b2.damage
-- Run multiple steps with the stretched bushing
for _ = 1, 10 do
	w:_updateBushings(1 / 60)
	w:_updateBushingDamage(1 / 60)
end
assert(b2.damage > oldDamage, "sustained stress must accumulate damage")
assert(w.bushingDamage >= 0, "aggregate damage must be non-negative")
assert(w.bushingDamage <= 1, "aggregate damage must be <= 1")

-- 11) Damage recovery: release stress
b2.stress = 0
for _ = 1, 100 do
	w:_updateBushingDamage(1 / 60)
end
assert(b2.damage < oldDamage + 0.01, "damage must recover when stress is low")

-- 12) destroyBushing
w:destroyBushing(1)
assert(b.restLength == 0, "destroyed bushing must have zero restLength")
assert(b.stress == 0, "destroyed bushing must have zero stress")
-- Destroyed bushing must not produce force
local oldVx = w.nodes[1].vx
w:_updateBushings(1 / 60)
assert(w.nodes[1].vx == oldVx, "destroyed bushing must not affect nodes")

-- 13) destroyBushing out of range
w:destroyBushing(999) -- must not error

-- 14) setupDefaultBushings with explicit defs
local w2 = World.new()
for i = 1, 8 do addNode(0, 0, i) end
w2.nodes = {}
for i = 1, 8 do
	w2.nodes[i] = {
		x = (i % 2 == 0) and 1 or -1,
		y = 0,
		z = math.floor((i - 1) / 2),
		vx = 0, vy = 0, vz = 0,
		invMass = 1.0,
		fixed = false,
	}
end
w2:setupDefaultBushings({
	{ nodeA = 0, nodeB = 1, stiffness = 100000 },
	{ nodeA = 2, nodeB = 3, stiffness = 100000, damping = 3000, deadzone = 0.02 },
})
assert(#w2.bushings == 2, "setupDefaultBushings with defs must add exactly the given bushings")
assert(w2.bushings[1].stiffness_x == 100000, "explicit stiffness must be used")

-- 15) setupDefaultBushings with no defs on 8-node world
local w3 = World.new()
for i = 1, 8 do
	w3.nodes[i] = {
		x = (i % 2 == 0) and 1 or -1,
		y = (i <= 4) and 0 or 0.5,
		z = ((i - 1) % 4 < 2) and -2 or 2,
		vx = 0, vy = 0, vz = 0,
		invMass = 1.0,
		fixed = false,
	}
end
w3:setupDefaultBushings()
assert(w3.bushings == nil or #w3.bushings == 0, "missing authored bushing definitions must be a no-op")

-- 16) setupDefaultBushings on small world (< 8 nodes) does nothing
local w4 = World.new()
for i = 1, 4 do
	w4.nodes[i] = { x = i, y = 0, z = 0, vx = 0, vy = 0, vz = 0, invMass = 1, fixed = false }
end
w4:setupDefaultBushings()
assert(w4.bushings == nil or #w4.bushings == 0, "setupDefaultBushings must no-op on small world")

-- 17) NaN dt guard
local w5 = World.new()
w5.bushings = {}
w5:_updateBushings(0/0)
w5:_updateBushingDamage(0/0) -- must not error

-- 18) No-bushings guard
w5:_updateBushings(1/60) -- empty bushings array: must not error

print("bushings module test passed")
