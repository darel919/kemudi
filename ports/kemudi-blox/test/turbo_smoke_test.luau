--[[ turbo module smoke test for Lune.
	Quick validation of config, state, update, torque, telemetry.
	Run: lune run test/turbo_smoke_test.luau
]]

local Turbo = require("../src/vehicle/turbo")

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
	tol = tol or 1e-4
	if math.abs(a - b) > tol then
		error(string.format("expected %.6f, got %.6f (diff %.6f)", b, a, math.abs(a - b)))
	end
end

local function check(cond, msg)
	if not cond then error(msg or "assertion failed") end
end

local function makeConfig(overrides)
	local values = { type="Turbo", max_boost_psi=14.0, spool_time=1.8, wastegate_open_psi=14.0, wastegate_duty_cycle=0.8, bov_threshold=0.2, bov_vent_rate=2.0, intercooler_efficiency=0.70, intercooler_pressure_loss=0.05, compressor_efficiency=0.75, anti_lag_enabled=false, anti_lag_spool_rate=2.0, anti_lag_exhaust_temp_rise=300.0, boost_leak_rate=0.0, boost_leak_pressure_loss=10.0, exhaust_energy_coeff=0.000015, torque_per_psi=0.0285714, reference_charge_temp=25.0, sc_efficiency=0.75, sc_torque_per_psi=0.03, sc_parasitic_loss=0.05, sc_boost_at_rpm={{800, 0.0}, {3000, 8.0}, {6000, 14.0}} }
	for key, value in pairs(overrides or {}) do values[key] = value end
	return Turbo.newConfig(values)
end

-- ============================================================
-- Config
-- ============================================================

print("=== Config ===")

test("newConfig returns all defaults", function()
	local c = makeConfig({})
	check(c.type == "Turbo", "type should be Turbo")
	approx(c.max_boost_psi, 14.0)
	approx(c.spool_time, 1.8)
	approx(c.wastegate_open_psi, 14.0)
	approx(c.intercooler_efficiency, 0.70)
	approx(c.exhaust_energy_coeff, 0.000015)
	approx(c.sc_efficiency, 0.75)
end)

test("newConfig overrides apply", function()
	local c = makeConfig({ max_boost_psi = 25.0, type = "Supercharger" })
	approx(c.max_boost_psi, 25.0)
	check(c.type == "Supercharger", "type should be Supercharger")
	-- other defaults still present
	approx(c.spool_time, 1.8)
end)

-- ============================================================
-- State
-- ============================================================

print("\n=== State ===")

test("newState returns zeroed defaults", function()
	local s = Turbo.newState()
	approx(s.boost_pressure, 0.0)
	approx(s.spool_position, 0.0)
	approx(s.wastegate_position, 0.0)
	approx(s.bov_position, 0.0)
	approx(s.charge_air_temp, 25.0)
	approx(s.exhaust_temp, 200.0)
	approx(s.compressor_outlet_temp, 80.0)
	check(s.is_boosting == false)
	check(s.anti_lag_active == false)
end)

-- ============================================================
-- Turbo update
-- ============================================================

print("\n=== Turbo Update ===")

test("NoType: boost stays zero", function()
	local c = makeConfig({ type = "None" })
	local s = Turbo.newState()
	Turbo.update(s, c, 4000, 1.0, 25, 0.016)
	approx(s.boost_pressure, 0.0)
	check(s.is_boosting == false)
end)

test("Turbo: spool builds with RPM and throttle", function()
	local c = makeConfig({})
	local s = Turbo.newState()
	-- Apply many small steps to let spool build
	for _ = 1, 200 do
		Turbo.update(s, c, 5000, 1.0, 25, 0.016)
	end
	check(s.spool_position > 0.5, string.format("spool should build, got %.4f", s.spool_position))
	check(s.boost_pressure > 0, string.format("boost should be > 0, got %.4f", s.boost_pressure))
	check(s.is_boosting == true)
end)

test("Turbo: no throttle → no exhaust energy → spool decays", function()
	local c = makeConfig({})
	local s = Turbo.newState()
	-- Build spool
	for _ = 1, 200 do
		Turbo.update(s, c, 5000, 1.0, 25, 0.016)
	end
	local spool_at_peak = s.spool_position
	-- Now zero throttle, let it decay
	for _ = 1, 200 do
		Turbo.update(s, c, 5000, 0.0, 25, 0.016)
	end
	check(s.spool_position < spool_at_peak, "spool should decay without throttle")
end)

test("Turbo: wastegate opens when boost exceeds target", function()
	local c = makeConfig({ wastegate_open_psi = 5.0, wastegate_duty_cycle = 1.0 })
	local s = Turbo.newState()
	for _ = 1, 400 do
		Turbo.update(s, c, 6000, 1.0, 25, 0.016)
	end
	-- With low wastegate_open_psi and full duty cycle, wastegate should open
	check(s.wastegate_position > 0.1, string.format("wastegate should open, got %.4f", s.wastegate_position))
	-- Boost should be capped near wastegate_open_psi
	check(s.boost_pressure < c.max_boost_psi, "wastegate should limit boost")
end)

test("Turbo: BOV triggers on sudden throttle drop", function()
	local c = makeConfig({ bov_threshold = 0.2 })
	local s = Turbo.newState()
	-- Build boost
	for _ = 1, 200 do
		Turbo.update(s, c, 5000, 1.0, 25, 0.016)
	end
	check(s.boost_pressure > 0)
	local boost_before = s.boost_pressure
	-- Sudden throttle release
	Turbo.update(s, c, 5000, 0.0, 25, 0.016)
	check(s.bov_position > 0, "BOV should open on sudden throttle drop")
end)

test("Turbo: anti-lag builds spool off-throttle", function()
	local c = makeConfig({ anti_lag_enabled = true, anti_lag_spool_rate = 5.0 })
	local s = Turbo.newState()
	-- Run off-throttle at high RPM
	for _ = 1, 100 do
		Turbo.update(s, c, 4000, 0.0, 25, 0.016)
	end
	check(s.anti_lag_active == true)
	check(s.spool_position > 0, string.format("anti-lag should build spool, got %.4f", s.spool_position))
end)

test("Turbo: boost leak drains pressure", function()
	local c = makeConfig({ boost_leak_rate = 1.0, boost_leak_pressure_loss = 10.0 })
	local s = Turbo.newState()
	-- Build boost then measure with leak
	for _ = 1, 300 do
		Turbo.update(s, c, 5000, 1.0, 25, 0.016)
	end
	local boost_with_leak = s.boost_pressure
	-- Compare against no-leak
	local c2 = makeConfig({ boost_leak_rate = 0.0 })
	local s2 = Turbo.newState()
	for _ = 1, 300 do
		Turbo.update(s2, c2, 5000, 1.0, 25, 0.016)
	end
	check(boost_with_leak < s2.boost_pressure,
		string.format("leak should reduce boost: leak=%.4f noleak=%.4f", boost_with_leak, s2.boost_pressure))
end)

test("Turbo: NaN inputs don't corrupt state", function()
	local c = makeConfig({})
	local s = Turbo.newState()
	Turbo.update(s, c, 0/0, 0/0, 0/0, 0/0)
	-- State should be finite
	check(s.boost_pressure == s.boost_pressure, "boost_pressure should be finite")
	check(s.spool_position == s.spool_position, "spool_position should be finite")
	check(s.charge_air_temp == s.charge_air_temp, "charge_air_temp should be finite")
	check(s.exhaust_temp == s.exhaust_temp, "exhaust_temp should be finite")
end)

-- ============================================================
-- Supercharger
-- ============================================================

print("\n=== Supercharger ===")

test("Supercharger: instant boost from RPM, no lag", function()
	local c = makeConfig({
		type = "Supercharger",
		max_boost_psi = 10.0,
	})
	local s = Turbo.newState()
	-- Single update should already produce boost
	Turbo.update(s, c, 4000, 1.0, 25, 0.016)
	check(s.boost_pressure > 0, string.format("SC should boost instantly, got %.4f", s.boost_pressure))
end)

test("Supercharger: RPM lookup interpolation", function()
	local c = makeConfig({ type = "Supercharger", max_boost_psi = 10.0 })
	local s = Turbo.newState()
	-- At 800 RPM → 0% boost
	Turbo.update(s, c, 800, 1.0, 25, 0.016)
	approx(s.boost_pressure, 0.0, 0.1)
	-- At 6000 RPM → 100% boost (before intercooler loss)
	local s2 = Turbo.newState()
	Turbo.update(s2, c, 6000, 1.0, 25, 0.016)
	check(s2.boost_pressure > s.boost_pressure, "higher RPM should mean more boost")
end)

-- ============================================================
-- Torque
-- ============================================================

print("\n=== Torque ===")

test("getEffectiveTorque: no boost → base torque", function()
	local c = makeConfig({})
	local s = Turbo.newState()
	local result = Turbo.getEffectiveTorque(s, c, 100.0)
	approx(result, 100.0)
end)

test("getEffectiveTorque: turbo boost multiplies torque", function()
	local c = makeConfig({ max_boost_psi = 14.0 })
	local s = Turbo.newState()
	s.boost_pressure = 14.0  -- full boost
	s.charge_air_temp = 25.0
	local result = Turbo.getEffectiveTorque(s, c, 100.0)
	-- At full boost: 100 * (1 + 1.0 * 0.40) * (1 + 0.01 * 0) = 140
	approx(result, 140.0, 0.1)
end)

test("getEffectiveTorque: supercharger lower gain + parasitic", function()
	local c = makeConfig({ type = "Supercharger", sc_parasitic_loss = 0.15 })
	local s = Turbo.newState()
	s.boost_pressure = 10.0
	s.charge_air_temp = 25.0
	local result = Turbo.getEffectiveTorque(s, c, 100.0)
	-- boost_ratio = 10/14 = 0.7143
	-- multiplier = 1 + 0.7143 * 0.30 = 1.2143
	-- parasitic: 100 * (1 - 0.15 * 0.7143) = 100 * 0.8929 = 89.29
	-- final = 89.29 * 1.2143 = ~108.4
	-- (intercooler density effect is zero since charge_temp == 25)
	check(result > 100 and result < 130, string.format("SC torque should gain modestly, got %.2f", result))
end)

test("getEffectiveTorque: intercooler cooling boosts power", function()
	local c = makeConfig({})
	local s = Turbo.newState()
	s.boost_pressure = 14.0
	s.charge_air_temp = 15.0  -- cooler than reference 25°C
	local result_cool = Turbo.getEffectiveTorque(s, c, 100.0)
	s.charge_air_temp = 35.0  -- hotter than reference
	local result_hot = Turbo.getEffectiveTorque(s, c, 100.0)
	check(result_cool > result_hot, "cooler charge should yield more torque")
end)

test("getEffectiveTorque: None type returns base", function()
	local c = makeConfig({ type = "None" })
	local s = Turbo.newState()
	s.boost_pressure = 14.0  -- shouldn't matter
	local result = Turbo.getEffectiveTorque(s, c, 100.0)
	approx(result, 100.0)
end)

test("getEffectiveTorque: NaN-safe", function()
	local c = makeConfig({})
	local s = Turbo.newState()
	s.boost_pressure = 0/0
	local result = Turbo.getEffectiveTorque(s, c, 100.0)
	check(result == result, "result should be finite")
end)

-- ============================================================
-- Telemetry
-- ============================================================

print("\n=== Telemetry ===")

test("telemetry returns all fields", function()
	local s = Turbo.newState()
	s.boost_pressure = 7.5
	s.spool_position = 0.6
	s.wastegate_position = 0.2
	s.charge_air_temp = 30.0
	s.exhaust_temp = 450.0
	local t = Turbo.telemetry(s)
	approx(t.boost_psi, 7.5)
	approx(t.spool, 0.6)
	approx(t.wastegate, 0.2)
	approx(t.charge_temp, 30.0)
	approx(t.exhaust_temp, 450.0)
end)

-- ============================================================
-- Summary
-- ============================================================

print(string.format("\n=== Results: %d/%d passed, %d failed ===", passed, total, failed))
if failed > 0 then
	error(string.format("%d test(s) FAILED", failed))
end
