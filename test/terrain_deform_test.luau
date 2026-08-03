-- Lune test for terrain_deform module.
-- Run: lune run test/terrain_deform_test.luau

local TerrainDeform = require("../src/physics/terrain_deform")
local Contract = require("../src/contract")
local Math = require("../src/math")

local function approx(actual, expected, tolerance, message)
	assert(math.abs(actual - expected) <= tolerance,
		message or string.format("expected %.8f, got %.8f", expected, actual))
end

-- Minimal World skeleton matching real init.luau fields
local gridSize = Contract.TERRAIN_GRID_SIZE
local World = {
	nodes = {},
	terrainRuts = {},
	suspension_states = nil,
	telemetry = {},
	fixedDt = 1 / 60,
}

-- Initialize rut grid (matches init.luau)
World.terrainRuts = table.create(gridSize * gridSize, 0)

-- Initialize telemetry array
World.telemetry = table.create(Contract.TELEMETRY_LEN, 0)

-- Install
TerrainDeform.install(World, Contract, Math)

-- Test 1: setupTerrainDeformation uses explicit vehicle/map data
assert(type(World.setupTerrainDeformation) == "function", "setupTerrainDeformation must be a function")
World.contact_patch_area = 0.03
World.surfaceHardness = 0.5
World.rutRecoveryRate = 0.002
World.maxRutDepth = 0.5
World.spreadFactor = 0.25
World.recoveryDepthDamping = 0.1
World.resistanceMax = 0.5
World:setupTerrainDeformation()
assert(World.surfaceHardness == 0.5, "default surfaceHardness must be 0.5")
assert(World.rutRecoveryRate == 0.002, "default rutRecoveryRate must be 0.002")
assert(World.maxRutDepth == 0.5, "default maxRutDepth must be 0.5")
print("  [1/8] setupTerrainDeformation defaults OK")

-- Test 2: all required methods attached
assert(type(World._updateTerrainDeformation) == "function", "missing _updateTerrainDeformation")
assert(type(World._recoverTerrainDeformation) == "function", "missing _recoverTerrainDeformation")
assert(type(World.setSurfaceHardness) == "function", "missing setSurfaceHardness")
assert(type(World.getRutResistance) == "function", "missing getRutResistance")
print("  [2/8] all methods attached")

-- Test 3: setSurfaceHardness clamps
World:setSurfaceHardness(0.3)
assert(World.surfaceHardness == 0.3, "setSurfaceHardness must store value")
World:setSurfaceHardness(-1.0)
assert(World.surfaceHardness == 0.0, "setSurfaceHardness must clamp to 0")
World:setSurfaceHardness(5.0)
assert(World.surfaceHardness == 1.0, "setSurfaceHardness must clamp to 1")
print("  [3/8] setSurfaceHardness clamping OK")

-- Test 4: getRutResistance returns 0 on flat terrain
World:setupTerrainDeformation()
local res = World:getRutResistance(0, 0)
assert(res == 0.0, "flat terrain must give zero resistance")
print("  [4/8] getRutResistance flat terrain OK")

-- Test 5: _updateTerrainDeformation deforms grid when wheel grounded
World:setupTerrainDeformation()
World:setSurfaceHardness(0.0) -- full soft, max deformation

-- Create 4 wheel nodes at origin
World.nodes = {
	{ x = 0, y = 0.3, z = 0, vx = 0, vy = 0, vz = 0, mass = 30, id = "wh_fl" },
	{ x = 0, y = 0.3, z = 0, vx = 0, vy = 0, vz = 0, mass = 30, id = "wh_fr" },
	{ x = 0, y = 0.3, z = 0, vx = 0, vy = 0, vz = 0, mass = 30, id = "wh_rl" },
	{ x = 0, y = 0.3, z = 0, vx = 0, vy = 0, vz = 0, mass = 30, id = "wh_rr" },
}

-- All 4 wheels grounded with load
World.suspension_states = {
	{ load = 5000, is_airborne = false, contact_point = { 0, 0.1, 0 } },
	{ load = 5000, is_airborne = false, contact_point = { 0, 0.1, 0 } },
	{ load = 5000, is_airborne = false, contact_point = { 0, 0.1, 0 } },
	{ load = 5000, is_airborne = false, contact_point = { 0, 0.1, 0 } },
}

-- Zero speed in telemetry for max speedFactor
World.telemetry[Contract.T.SPEED_MPS + 1] = 0

World:_updateTerrainDeformation(1 / 60)

-- Grid center: gx=gz=32, idx = 32*64+32+1 = 2081
local centerIdx = gridSize * (gridSize / 2) + (gridSize / 2) + 1
assert(World.terrainRuts[centerIdx] > 0, "center cell must be deformed after wheel contact")
assert(World.terrainRuts[centerIdx] <= World.maxRutDepth, "deformation must not exceed maxRutDepth")
print("  [5/8] _updateTerrainDeformation writes grid OK")

-- Test 6: airborne wheels don't deform
local beforeDeform = World.terrainRuts[centerIdx]
World.suspension_states[1].is_airborne = true
World.suspension_states[2].is_airborne = true
World.suspension_states[3].is_airborne = true
World.suspension_states[4].is_airborne = true
World:_updateTerrainDeformation(1 / 60)
assert(World.terrainRuts[centerIdx] == beforeDeform,
	"airborne wheels must not add deformation")
print("  [6/8] airborne wheels skip deformation OK")

-- Test 7: _recoverTerrainDeformation reduces depth over time
World:setupTerrainDeformation()
World.suspension_states[1].is_airborne = false
World.suspension_states[1].load = 5000
World.suspension_states[1].contact_point = { 0, 0.1, 0 }

-- Set a known rut depth
World.terrainRuts[centerIdx] = 0.1
local depthBefore = World.terrainRuts[centerIdx]
World:_recoverTerrainDeformation(1.0) -- 1 second recovery
local depthAfter = World.terrainRuts[centerIdx]
assert(depthAfter < depthBefore, "recovery must reduce rut depth")
assert(depthAfter > 0, "1s recovery at 0.1m depth must not fully recover")
print("  [7/8] _recoverTerrainDeformation OK")

-- Test 8: getRutResistance proportional to depth
World:setupTerrainDeformation()
World.terrainRuts[centerIdx] = 0.25 -- half of maxRutDepth=0.5
local resistance = World:getRutResistance(0, 0)
approx(resistance, 0.25, 1e-6, "50% depth must give 50% of max resistance (0.25)")

World.terrainRuts[centerIdx] = World.maxRutDepth
local maxResistance = World:getRutResistance(0, 0)
assert(maxResistance <= 0.5 + 1e-9, "resistance must be capped at 0.5")
print("  [8/8] getRutResistance proportional OK")

print("terrain_deform_test passed")
