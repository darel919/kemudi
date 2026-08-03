-- Physics integration regression tests for Lune.
-- Run: lune run test/physics_integration_test.luau

local process = require("@lune/process")
local Integration = require("../src/physics/integration")
local Math = require("../src/math")

local passed = 0
local failed = 0

local function test(name, fn)
	local ok, err = pcall(fn)
	if ok then
		passed += 1
		print(string.format("  PASS  %s", name))
	else
		failed += 1
		print(string.format("  FAIL  %s: %s", name, tostring(err)))
	end
end

test("world step uses finite math helpers", function()
	local World = {}
	Integration.install(World, Math)
	local world = {
		nodes = {
			{ x = 0, y = 1, z = 0, vx = 0, vy = 0, vz = 0, fx = 0, fy = 10, fz = 0, invMass = 0.1, fixed = false, lastForce = 0 },
		},
	}
	World._integrateVelocities(world, 1 / 60)
end)

print(string.format("\n=== Results: %d/%d passed ===", passed, passed + failed))
if failed > 0 then
	process.exit(1)
end
