--[[ brakes unit tests for Lune.
	Validates the brake thermal model, fade, force calc, and fluid pressure.
	Run: lune run test/brakes_test.luau
--]]

local process = require("@lune/process")
local Brakes = require("../src/vehicle/brakes")

local passed = 0
local failed = 0
local total = 0

local function test(name, fn)
	total += 1
	local ok, err = pcall(fn)
	if ok then
		passed += 1
		print(string.format("  PASS  %s", name))
	else
		failed += 1
		print(string.format("  FAIL  %s: %s", name, tostring(err)))
	end
end

local function approx(a, b, tol)
	tol = tol or 1e-6
	if math.abs(a - b) > tol then
		error(string.format("expected %.6f, got %.6f (diff %.6f)", b, a, math.abs(a - b)))
	end
end

local function check(cond, msg)
	if not cond then error(msg or "assertion failed") end
end

local function makeConfig(overrides)
	local values = {
		bias_front = 0.60, disc_mass = 5.0, disc_specific_heat = 500.0, disc_area = 0.02,
		convective_coeff = 75.0, pad_friction_nominal = 0.42, fluid_boil_temp = 300.0,
		fade_onset_temp = 550.0, fade_critical_temp = 800.0, max_disc_temp = 1000.0,
		brake_torque_max = 2500.0, contact_patch_radius = 0.4385, airflow_speed_reference = 50.0, fluid_fade_end_temp = 500.0,
	}
	for key, value in pairs(overrides or {}) do values[key] = value end
	return Brakes.newConfig(values)
end

local function runUpdate(states, config, speed, ambient, dt)
	for _, state in ipairs(states) do
		if state._last_brake_torque == nil then state._last_brake_torque = config.brake_torque_max * math.max(config.pad_friction_nominal, 0) end
	end
	return Brakes["update"](states, config, speed, ambient, dt)
end

-- ============================================================
-- Factory defaults
-- ============================================================

print("=== Factory Defaults ===")

test("newState returns all required fields", function()
	local s = Brakes.newState()
	approx(s.disc_temp, 25.0)
	approx(s.pad_temp, 25.0)
	approx(s.fluid_temp, 25.0)
	approx(s.fade_factor, 1.0)
	approx(s.peak_temp_seen, 25.0)
	approx(s.total_energy_dissipated, 0.0)
end)

test("newConfig returns all required fields", function()
	local c = makeConfig()
	approx(c.bias_front, 0.60)
	approx(c.disc_mass, 5.0)
	approx(c.disc_specific_heat, 500.0)
	approx(c.disc_area, 0.02)
	approx(c.convective_coeff, 75.0)
	approx(c.pad_friction_nominal, 0.42)
	approx(c.fluid_boil_temp, 300.0)
	approx(c.fade_onset_temp, 550.0)
	approx(c.fade_critical_temp, 800.0)
	approx(c.max_disc_temp, 1000.0)
	approx(c.brake_torque_max, 2500.0)
	approx(c.contact_patch_radius, 0.4385)
end)

test("newConfig accepts overrides", function()
	local c = makeConfig({ disc_mass = 8.0, bias_front = 0.70 })
	approx(c.disc_mass, 8.0)
	approx(c.bias_front, 0.70)
	-- Other defaults intact
	approx(c.brake_torque_max, 2500.0)
end)

-- ============================================================
-- Thermal model
-- ============================================================

print("\n=== Thermal Model ===")

test("Heat generation at high speed", function()
	local s = Brakes.newState()
	local c = makeConfig()
	local before = s.disc_temp
	runUpdate({ s }, c, 100, 25, 1 / 60)
	check(s.disc_temp > before, "disc should heat up when braking at speed")
end)

test("Convective cooling dominates at low speed", function()
	local s = Brakes.newState()
	s.disc_temp = 200
	local c = makeConfig()
	-- Stationary car: no brake heat generation, only cooling
	runUpdate({ s }, c, 0, 25, 1 / 60)
	check(s.disc_temp < 200, "disc should cool when stationary")
end)

test("Airflow factor increases with speed", function()
	local s1 = Brakes.newState()
	s1.disc_temp = 200
	local s2 = Brakes.newState()
	s2.disc_temp = 200
	-- Zero friction to isolate cooling from heat generation
	local c = makeConfig({ pad_friction_nominal = 0 })
	runUpdate({ s1 }, c, 5, 25, 1 / 60)   -- slow: airflow ~0.42
	runUpdate({ s2 }, c, 50, 25, 1 / 60)  -- fast: airflow 1.0
	check(s2.disc_temp < s1.disc_temp, "faster car should cool disc more")
end)

test("Pad temp lags disc temp", function()
	local s = Brakes.newState()
	local c = makeConfig()
	-- Heat disc rapidly
	for _ = 1, 300 do
		runUpdate({ s }, c, 100, 25, 1 / 60)
	end
	check(s.disc_temp > s.pad_temp, "pad should lag behind disc temperature")
end)

test("Fluid temp lags pad temp", function()
	local s = Brakes.newState()
	local c = makeConfig()
	for _ = 1, 600 do
		runUpdate({ s }, c, 100, 25, 1 / 60)
	end
	check(s.pad_temp > s.fluid_temp, "fluid should lag behind pad temperature")
end)

test("Disc temp clamped to max_disc_temp", function()
	local s = Brakes.newState()
	s.disc_temp = 1500 -- above max
	local c = makeConfig()
	runUpdate({ s }, c, 0, 25, 1 / 60) -- cool toward ambient
	check(s.disc_temp <= c.max_disc_temp, "disc temp should not exceed max_disc_temp after update")
end)

test("Peak temp tracked", function()
	local s = Brakes.newState()
	local c = makeConfig()
	runUpdate({ s }, c, 100, 25, 1 / 60)
	check(s.peak_temp_seen >= s.disc_temp, "peak should be >= current temp")
end)

test("Total energy dissipated accumulates", function()
	local s = Brakes.newState()
	local c = makeConfig()
	runUpdate({ s }, c, 50, 25, 1 / 60)
	check(s.total_energy_dissipated > 0, "energy should accumulate over time")
end)

test("Zero dt is a no-op", function()
	local s = Brakes.newState()
	s.disc_temp = 50
	local c = makeConfig()
	runUpdate({ s }, c, 100, 25, 0)
	approx(s.disc_temp, 50)
end)

-- ============================================================
-- Fade
-- ============================================================

print("\n=== Fade ===")

test("No fade below onset temp", function()
	local s = Brakes.newState()
	local c = makeConfig()
	s.disc_temp = 500 -- below fade_onset_temp (550)
	-- Trigger fade calc via update
	runUpdate({ s }, c, 0, 25, 1 / 60)
	check(s.fade_factor >= 0.99, "no fade below onset temp")
end)

test("Fade active above onset temp", function()
	local s = Brakes.newState()
	s.disc_temp = 650 -- between onset (550) and critical (800)
	local c = makeConfig()
	-- Update recalculates fade
	runUpdate({ s }, c, 0, 25, 1 / 60)
	check(s.fade_factor < 1.0, "fade should reduce friction above onset")
	check(s.fade_factor > 0.15, "fade should not be at minimum in this range")
end)

test("Severe fade at critical temp", function()
	local s = Brakes.newState()
	s.disc_temp = 850 -- above critical; stays above after 1-frame cooling
	local c = makeConfig()
	runUpdate({ s }, c, 0, 25, 1 / 60)
	check(s.fade_factor <= 0.3, "fade should be at 0.3 at critical temp")
	check(s.fade_factor >= 0.15, "fade should not drop below 0.15")
end)

test("Fluid boil causes additional fade", function()
	local s = Brakes.newState()
	s.disc_temp = 600 -- mechanical fade active
	s.fluid_temp = 350 -- above boil (300)
	local c = makeConfig()
	runUpdate({ s }, c, 0, 25, 1 / 60)
	-- fade = mechanical (~0.88) * 0.3 = ~0.26
	check(s.fade_factor < 0.5, "fluid boil should cause severe fade")
end)

-- ============================================================
-- Force calculation
-- ============================================================

print("\n=== Force Calculation ===")

test("Zero input = zero force", function()
	local s = Brakes.newState()
	local c = makeConfig()
	local forces = Brakes.calculateForces({ s }, c, 0, 0.3, { 50, 50, 50, 50 })
	approx(forces[1], 0)
end)

test("Full brake splits by bias", function()
	local states = {}
	for _ = 1, 4 do table.insert(states, Brakes.newState()) end
	local c = makeConfig()
	local forces = Brakes.calculateForces(states, c, 1.0, 0.3, { 50, 50, 50, 50 })
	-- front (wheels 1-2): 1.0 * 0.60 * 2500 = 1500
	-- rear (wheels 3-4): 1.0 * 0.40 * 2500 = 1000
	approx(forces[1], 1500)
	approx(forces[2], 1500)
	approx(forces[3], 1000)
	approx(forces[4], 1000)
end)

test("Fade reduces brake force", function()
	local s1 = Brakes.newState()
	s1.fade_factor = 0.5
	local c = makeConfig()
	local forces = Brakes.calculateForces({ s1 }, c, 1.0, 0.3, { 50 })
	-- 1.0 * 0.60 * 2500 * 0.5 = 750
	approx(forces[1], 750)
end)

test("Four-wheel force output", function()
	local states = {}
	for _ = 1, 4 do table.insert(states, Brakes.newState()) end
	local c = makeConfig()
	local forces = Brakes.calculateForces(states, c, 0.5, 0.3, { 20, 20, 20, 20 })
	check(#forces == 4, "should return 4 forces")
	check(forces[1] > 0, "front wheels should have force")
	check(forces[3] > 0, "rear wheels should have force")
end)

test("Input clamped to [0,1]", function()
	local s = Brakes.newState()
	local c = makeConfig()
	local f1 = Brakes.calculateForces({ s }, c, -1, 0.3, { 50 })
	local f2 = Brakes.calculateForces({ s }, c, 2.0, 0.3, { 50 })
	approx(f1[1], 0)
	approx(f2[1], 1500)
end)

-- ============================================================
-- Fluid pressure
-- ============================================================

print("\n=== Fluid Pressure ===")

test("Full pressure when cool", function()
	local s = Brakes.newState()
	local c = makeConfig()
	local p = Brakes.getEffectivePressure(s, c)
	approx(p, 1.0, 0.01)
end)

test("Pressure drops near boil", function()
	local s = Brakes.newState()
	s.fluid_temp = 350
	s.fade_factor = 1.0
	local c = makeConfig()
	local p = Brakes.getEffectivePressure(s, c)
	check(p < 1.0, "pressure should drop above boil temp")
	check(p > 0, "pressure should not be zero at 350C")
end)

test("Pressure approaches zero well above boil", function()
	local s = Brakes.newState()
	s.fluid_temp = 500
	s.fade_factor = 1.0
	local c = makeConfig()
	local p = Brakes.getEffectivePressure(s, c)
	check(p < 0.1, "pressure should be near zero far above boil")
end)

test("Null state returns 0", function()
	local p = Brakes.getEffectivePressure(nil)
	approx(p, 0)
end)

-- ============================================================
-- Integration: sustained heavy braking
-- ============================================================

print("\n=== Integration: Sustained Braking ===")

test("Prolonged full brake causes fade", function()
	local s = Brakes.newState()
	local c = makeConfig()
	-- Simulate 60 seconds of full braking at 100 m/s
	for _ = 1, 3600 do
		runUpdate({ s }, c, 100, 25, 1 / 60)
	end
	check(s.disc_temp > 100, "disc should be very hot after sustained braking")
	check(s.fade_factor < 1.0, "should have fade after sustained braking")
end)

test("Cooling at standstill recovers temperature", function()
	local s = Brakes.newState()
	s.disc_temp = 300
	-- Zero friction to isolate cooling; car moving for full airflow
	local c = makeConfig({ pad_friction_nominal = 0 })
	-- Cool for 3000 seconds at 30 m/s (full airflow factor)
	for _ = 1, 3000 do
		runUpdate({ s }, c, 30, 25, 1)
	end
	check(s.disc_temp < 100, "disc should cool below 100C over extended time")
	check(s.fade_factor >= 0.99, "fade should recover when cool")
end)

-- ============================================================
-- No NaN / inf values after update
-- ============================================================

print("\n=== Numerical Safety ===")

test("No NaN after extreme inputs", function()
	local s = Brakes.newState()
	local c = makeConfig()
	-- Extreme speed, extreme dt
	runUpdate({ s }, c, 10000, 25, 1 / 60)
	check(s.disc_temp == s.disc_temp, "disc_temp should not be NaN")
	check(s.fade_factor == s.fade_factor, "fade_factor should not be NaN")
	-- Check all finite
	check(s.disc_temp ~= math.huge and s.disc_temp ~= -math.huge, "disc_temp should not be infinite")
end)

test("No NaN from calculateForces with zero states", function()
	local forces = Brakes.calculateForces({}, makeConfig(), 0.5, 0.3, {})
	check(#forces == 0, "empty input returns empty output")
end)

-- ============================================================
-- Summary
-- ============================================================

print(string.format("\n=== Results: %d/%d passed ===", passed, total))
if failed > 0 then
	process.exit(1)
end
