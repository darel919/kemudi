--[[
	Generic fixture runner for the Kemudi physics engine.
	Loads any fixture JSON from fixtures/engine/, replays steps, checks assertions.
	Run with: lune run test/fixture_runner.luau [fixture_name]
	Without args: runs all fixtures in fixtures/engine/.

Uses the real engine World from src/physics/init.luau — no inlined copy.
--]]

local fs = require("@lune/fs")
local serde = require("@lune/serde")
local process = require("@lune/process")

local World = require("../src/physics")

-- ============================================================
-- Resolve a dotted path like "nodes[0].positionM[1]"
-- ============================================================

local FIELD_MAP = {
	positionM = { "x", "y", "z" },
	velocityMps = { "vx", "vy", "vz" },
}

local function resolve(path, state)
	local val = state
	for token in path:gmatch("[^%.]+") do
		local key, idx = token:match("^(%w+)%[(%d+)%]")
		if key and idx then
			idx = tonumber(idx)
			if key == "nodes" or key == "beams" then
				val = val[key]
				if val == nil then return nil end
				val = val[idx + 1]
			elseif key == "telemetry" then
				val = val[key]
				if val == nil then return nil end
				val = val[idx + 1]
			else
				local mapping = FIELD_MAP[key]
				if mapping then
					val = val[mapping[idx + 1]]
				else
					val = val[key]
					if val == nil then return nil end
					val = val[idx]
				end
			end
		else
			val = val[token]
		end
		if val == nil then return nil end
	end
	return val
end

local function assert_check(actual, op, expected, tol, desc)
	if actual == nil then return false, "path resolved to nil" end
	if op == "equals" then
		if actual ~= expected then return false, string.format("expected %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "approximatelyEquals" then
		if math.abs(actual - expected) > (tol or 1e-6) then
			return false, string.format("expected ~%.10f, got %.10f (tol %.10f)", expected, actual, tol or 1e-6)
		end
	elseif op == "lessThan" then
		if not (actual < expected) then return false, string.format("expected < %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "greaterThan" then
		if not (actual > expected) then return false, string.format("expected > %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "lessThanOrEquals" then
		if not (actual <= expected) then return false, string.format("expected <= %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "greaterThanOrEquals" then
		if not (actual >= expected) then return false, string.format("expected >= %s, got %s", tostring(expected), tostring(actual)) end
	else
		return false, "unknown operator: " .. tostring(op)
	end
	return true, ""
end

-- ============================================================
-- Load fixture, simulate, check assertions
-- ============================================================

local function runFixture(path)
	local raw = fs.readFile(path)
	local fixture = serde.decode("json", raw)

	local world = World.new()
	if fixture.initial then
		world.gravity = fixture.initial.gravityMps2 or -9.81
		world.fixedDt = fixture.initial.fixedDtSeconds or (1/60)

		for _, n in fixture.initial.nodes do
			world:addNode(n.id, n.positionM[1], n.positionM[2], n.positionM[3],
				n.massKg or 1, n.fixed or false, n.collision)
		end
		for _, b in (fixture.initial.beams or {}) do
			world:addBeam(b.id, b.nodeA, b.nodeB,
				b.stiffness or 10000, b.damping or 100, b.strength or 5000)
		end
		for _, tri in (fixture.initial.triangles or {}) do
			world:addTriangle(tri.a, tri.b, tri.c)
		end
	end

	-- Replay steps
	for _, step in (fixture.steps or {}) do
		local dt = step.elapsedSeconds or world.fixedDt
		if step.controls then
			for k, v in pairs(step.controls) do
				world.controls[k] = v
			end
		end
		world:step(dt)
	end

	-- Check assertions
	local pass, fail = 0, 0
	for _, a in (fixture.assertions or {}) do
		local val = resolve(a.path, world)
		local ok, msg = assert_check(val, a.operator, a.value, a.absoluteTolerance, a.description)
		if ok then
			pass += 1
		else
			fail += 1
			print(string.format("    FAIL: %s — %s (got %s)", a.path, msg, tostring(val)))
		end
	end

	return pass, fail
end

-- ============================================================
-- Main
-- ============================================================

local FIXTURES_DIR = "fixtures/engine"

local fixturePaths = {}
local args = process.args
if #args > 2 then
	for i = 3, #args do
		table.insert(fixturePaths, FIXTURES_DIR .. "/" .. args[i] .. ".json")
	end
else
	local entries = fs.readDir(FIXTURES_DIR)
	for _, entry in entries do
		if entry:match("%.json$") then
			table.insert(fixturePaths, FIXTURES_DIR .. "/" .. entry)
		end
	end
	table.sort(fixturePaths)
end

print("=== Kemudi Engine Fixture Runner ===\n")

local totalPass, totalFail = 0, 0
local fixturePass, fixtureFail = 0, 0

for _, fpath in fixturePaths do
	local name = fpath:match("([^/]+)%.json$")
	io = io or {}
	local ok, p, f = pcall(function()
		return runFixture(fpath)
	end)
	if ok then
		totalPass += p; totalFail += f
		if f == 0 then
			print(string.format("PASS (%d/%d)", p, p))
			fixturePass += 1
		else
			print(string.format("FAIL (%d/%d)", p, p + f))
			fixtureFail += 1
		end
	else
		print("ERROR: " .. tostring(p))
		fixtureFail += 1
		totalFail += 1
	end
end

print(string.format("\n=== Results: %d/%d fixtures passed, %d/%d assertions passed ===",
	fixturePass, fixturePass + fixtureFail, totalPass, totalPass + totalFail))
if totalFail > 0 or fixtureFail > 0 then
	process.exit(1)
end
