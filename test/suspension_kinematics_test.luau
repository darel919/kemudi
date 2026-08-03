local SK = require("../src/vehicle/suspension_kinematics")

local function makeConfig(overrides)
	local values = { camber_gain=-0.15, camber_at_rest=-0.5, toe_gain=0.05, toe_at_rest=0.1, roll_center_height=0.15, roll_center_migration=0.002, anti_dive_pct=0.15, anti_squat_pct=0.20, kingpin_inclination=12.0, scrub_radius=0.02, caster_angle=5.0, max_compression=80.0, max_extension=60.0 }
	for key, value in pairs(overrides or {}) do values[key] = value end
	return SK.newConfig(values)
end

local passed = 0
local failed = 0
local total = 0

local function test(name, fn)
	total = total + 1
	local ok, err = pcall(fn)
	if ok then
		passed = passed + 1
		print("  PASS  " .. name)
	else
		failed = failed + 1
		print("  FAIL  " .. name .. ": " .. tostring(err))
	end
end

local function approx(a, b, tol)
	tol = tol or 1e-6
	if math.abs(a - b) > tol then
		error("expected " .. string.format("%.6f", b) .. ", got " .. string.format("%.6f", a) .. " (diff " .. string.format("%.6f", math.abs(a - b)) .. ")")
	end
end

print("=== SuspensionKinematics Tests ===")

-- Config
test("newConfig defaults", function()
	local cfg = makeConfig()
	approx(cfg.camber_gain, -0.15)
	approx(cfg.anti_dive_pct, 0.15)
	approx(cfg.max_compression, 80.0)
end)

test("newConfig overrides merge", function()
	local cfg = makeConfig({camber_at_rest = -2.0})
	approx(cfg.camber_at_rest, -2.0)
	approx(cfg.camber_gain, -0.15)
end)

-- Camber
test("getCamber at rest", function()
	local cfg = makeConfig()
	approx(SK.getCamber(cfg, 0), -0.5)
end)

test("getCamber 10mm compression", function()
	local cfg = makeConfig()
	approx(SK.getCamber(cfg, 10), -2.0)
end)

test("getCamber clamp negative", function()
	local cfg = makeConfig()
	local c = SK.getCamber(cfg, 100)
	approx(c, -8)
end)

test("getCamber clamp positive", function()
	local cfg = makeConfig({camber_gain = 0.1, camber_at_rest = 0})
	local c = SK.getCamber(cfg, 100)
	approx(c, 8)
end)

-- Toe
test("getToe at rest", function()
	local cfg = makeConfig()
	approx(SK.getToe(cfg, 0), 0.1)
end)

test("getToe 10mm compression", function()
	local cfg = makeConfig()
	approx(SK.getToe(cfg, 10), 0.6)
end)

test("getToe clamp", function()
	local cfg = makeConfig()
	approx(SK.getToe(cfg, 1000), 3)
end)

-- Roll center
test("getRollCenter height and migration", function()
	local cfg = makeConfig()
	local rc = SK.getRollCenter(cfg, 5)
	approx(rc.height, 0.15)
	approx(rc.lateral_offset, 0.01)
end)

-- Anti-dive
test("getAntiDiveForce scales with h/wb", function()
	local cfg = makeConfig()
	approx(SK.getAntiDiveForce(cfg, 1000, 0.5, 2.5), 30)
end)

test("getAntiDiveForce wb guard", function()
	local cfg = makeConfig()
	approx(SK.getAntiDiveForce(cfg, 1000, 0.5, 0), 0)
end)

-- Anti-squat
test("getAntiSquatForce", function()
	local cfg = makeConfig()
	approx(SK.getAntiSquatForce(cfg, 1000, 0.5, 2.5), 40)
end)

-- Scrub radius
test("getScrubRadius positive at rest", function()
	local cfg = makeConfig()
	local sr = SK.getScrubRadius(cfg, 0)
	approx(sr, 0.02)
end)

test("getScrubRadius changes with travel", function()
	local cfg = makeConfig()
	local sr50 = SK.getScrubRadius(cfg, 50)
	local sr0 = SK.getScrubRadius(cfg, 0)
	if sr50 <= sr0 then error("scrub should increase with compression") end
end)

-- Apply kinematics
test("applyKinematics writes camber/toe/scrub", function()
	local cfg = makeConfig()
	local state = { compression = 0.125 }
	SK.applyKinematics(state, cfg, 0)
	approx(state.camber, -2.0)
	approx(state.toe, 0.6)
	if state.scrub_radius == nil then error("scrub_radius not set") end
end)

test("applyKinematics nil state no error", function()
	SK.applyKinematics(nil, makeConfig(), 0)
end)

-- NaN/Inf guards
test("getCamber NaN guarded", function()
	local cfg = makeConfig()
	approx(SK.getCamber(cfg, 0/0), -0.5)
end)

test("getToe Inf guarded", function()
	local cfg = makeConfig()
	local t = SK.getToe(cfg, math.huge)
	if t > 3 or t < -3 then error("toe not clamped") end
end)

print("\n=== Results: " .. passed .. "/" .. total .. " passed ===")
if failed > 0 then error("Some tests failed") end