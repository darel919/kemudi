local Grid = require("../src/runtime/ground_zero_grid")

local function assertEqual(actual, expected, message)
	assert(actual == expected, string.format("%s: expected %s, got %s", message, tostring(expected), tostring(actual)))
end

local size = 512
local x, z = Grid.cellForPosition(0, 0, size)
assertEqual(x, 0, "origin x cell")
assertEqual(z, 0, "origin z cell")

x, z = Grid.cellForPosition(-0.01, -512.01, size)
assertEqual(x, -1, "negative x cell")
assertEqual(z, -2, "negative z cell")

assertEqual(Grid.key(-3, 7), "-3:7", "stable chunk key")

local centerX, centerY, centerZ = Grid.center(2, -1, size, -0.5)
assertEqual(centerX, 1280, "chunk center x")
assertEqual(centerY, -0.5, "chunk center y")
assertEqual(centerZ, -256, "chunk center z")

assert(Grid.isWithin(2, 2, 0, 0, 2), "edge chunk should be within radius")
assert(not Grid.isWithin(3, 0, 0, 0, 2), "outside chunk should not be within radius")

local cells = Grid.cellsAround(0, 0, 1)
assertEqual(#cells, 9, "radius-one chunk count")
local seen = {}
for _, cell in cells do
	assert(not seen[cell.key], "cellsAround must not duplicate keys")
	seen[cell.key] = true
end

print("GroundZero grid tests passed")
