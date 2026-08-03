local Chassis = require("../src/runtime/chassis")

local function node(x, y, z)
	return { x = x, y = y, z = z }
end

local nodes = {
	node(-1, 0.2, -2), node(1, 0.2, -2),
	node(-1, 0.2, 2), node(1, 0.2, 2),
	node(-1, 1.2, -1.8), node(1, 1.2, -1.8),
	node(-1, 1.2, 1.8), node(1, 1.2, 1.8),
}
local frame = Chassis.frameFromNodes(nodes)
assert(frame, "a valid four-mount chassis must produce a frame")

local function dot(a, b)
	return a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
end
local function length(v)
	return math.sqrt(dot(v, v))
end
assert(math.abs(length(frame.forward) - 1) < 1e-9)
assert(math.abs(length(frame.right) - 1) < 1e-9)
assert(math.abs(length(frame.up) - 1) < 1e-9)
assert(math.abs(dot(frame.forward, frame.right)) < 1e-9)
assert(math.abs(dot(frame.forward, frame.up)) < 1e-9)
assert(math.abs(dot(frame.right, frame.up)) < 1e-9)
assert(frame.forward[3] < -0.99, "front-to-rear ordering must preserve vehicle forward")
assert(math.abs(frame.position[3]) < 1e-9, "body frame must use the upper cage center")

-- A rolled/pitched chassis must still return an orthonormal frame rather than
-- silently falling back to a horizontal yaw-only orientation.
nodes[1].y += 0.4
nodes[2].y += 0.4
nodes[5].y += 0.4
nodes[6].y += 0.4
local pitched = Chassis.frameFromNodes(nodes)
assert(pitched and math.abs(pitched.forward[2]) > 0.01, "mount pitch must reach the presentation frame")
assert(math.abs(length(pitched.up) - 1) < 1e-9)

print("chassis frame tests passed")
