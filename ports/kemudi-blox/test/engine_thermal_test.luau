--[[ engine_thermal unit tests for Lune.
	Validates the ported thermal, lubrication, damage, and stress models.
	Run: lune run test/engine_thermal_test.luau
--]]

local process = require("@lune/process")
local EngineThermal = require("../src/vehicle/engine_thermal")

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

-- ============================================================
-- Thermal model
-- ============================================================

print("=== Thermal Model ===")

test("Thermal defaults: coolant at 20C", function()
	local t = EngineThermal.newThermal()
	approx(t.coolant_temp, 20)
end)

test("Heat generation at idle RPM", function()
	local t = EngineThermal.newThermal()
	local cooling = EngineThermal.newCoolingSystem()
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()

	-- Idle: 800 RPM, 0 throttle, 25C ambient
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 800, 7000, 0, 0, 0, 1, 1.0)
	check(t.coolant_temp > 20, "coolant should warm up at idle")
	check(t.oil_temp > 20, "oil should warm up at idle")
end)

test("Heat generation under load", function()
	local t = EngineThermal.newThermal()
	local cooling = EngineThermal.newCoolingSystem()
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()

	-- Simulate 100 seconds at 5000 RPM, full throttle
	for _ = 1, 6000 do
		EngineThermal.updateFull(t, cooling, lube, damage, stress, 5000, 7000, 1.0, 0, 0, 1, 1/60)
	end
	check(t.coolant_temp > 25, "coolant should rise significantly under load")
	check(t.oil_temp > 22, "oil should warm up under load")
	check(t.transmission_temp > 20, "trans temp should track oil")
end)

test("Cooling brings temp down with airflow", function()
	local t = EngineThermal.newThermal()
	t.coolant_temp = 90
	local cooling = EngineThermal.newCoolingSystem()
	local rate = EngineThermal.coolingRate(cooling, 90, 30)
	check(rate > 0, "should have cooling at 90C with speed")
end)

test("Thermostat closed when cold", function()
	local cooling = EngineThermal.newCoolingSystem()
	local valve = EngineThermal.thermostatValve(cooling, 70)
	approx(valve, 0)
end)

test("Thermostat open when hot", function()
	local cooling = EngineThermal.newCoolingSystem()
	local valve = EngineThermal.thermostatValve(cooling, 100)
	approx(valve, 1, 0.01)
end)

-- ============================================================
-- Lubrication
-- ============================================================

print("\n=== Fan Hysteresis ===")

test("Fan off during cold start", function()
	local t = EngineThermal.newThermal()
	local cooling = EngineThermal.newCoolingSystem()
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 800, 7000, 0, 0, 0, 1, 1/60)
	check(not cooling.fan_active, "fan should be OFF at cold start (20C)")
	approx(cooling.fan_speed, 0)
end)

test("Fan turns on above activation temp", function()
	local cooling = EngineThermal.newCoolingSystem()
	-- Simulate temp rising above 95C
	local t = EngineThermal.newThermal()
	t.coolant_temp = 96
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 3000, 7000, 0.8, 0, 0, 1, 1/60)
	check(cooling.fan_active, "fan should be ON above 95C")
	approx(cooling.fan_speed, 1.0, 0.01)
end)

test("Fan stays on until below deactivation temp (hysteresis)", function()
	local cooling = EngineThermal.newCoolingSystem()
	local t = EngineThermal.newThermal()
	t.coolant_temp = 92 -- between 90 (off) and 95 (on)
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()
	-- Fan was activated earlier
	cooling.fan_active = true
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 800, 7000, 0, 0, 0, 1, 1/60)
	check(cooling.fan_active, "fan should stay ON at 92C (above 90C deactivation)")
	-- Now drop below deactivation
	t.coolant_temp = 89
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 800, 7000, 0, 0, 0, 1, 1/60)
	check(not cooling.fan_active, "fan should turn OFF below 90C")
end)

test("Fan proportional speed between thresholds", function()
	local cooling = EngineThermal.newCoolingSystem()
	-- Force fan active, temp at midpoint between 90 and 95
	cooling.fan_active = true
	local speed_range = cooling.fan_activation_temp - cooling.fan_deactivation_temp
	local mid_temp = cooling.fan_deactivation_temp + speed_range * 0.5
	local t = EngineThermal.newThermal()
	t.coolant_temp = mid_temp
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 800, 7000, 0, 0, 0, 1, 1/60)
	approx(cooling.fan_speed, 0.5, 0.01)
end)

print("\n=== Heat Soak ===")

test("No heat soak at idle (engine warming up)", function()
	local t = EngineThermal.newThermal()
	t.coolant_temp = 40 -- warmed up from ambient
	local cooling = EngineThermal.newCoolingSystem()
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()
	local before = t.coolant_temp
	-- Idle: 800 RPM, 0 throttle
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 800, 7000, 0, 0, 0, 1, 1/60)
	check(t.coolant_temp >= before, "coolant should not cool down at idle (no heat soak)")
end)

test("Heat soak only when engine OFF", function()
	local t = EngineThermal.newThermal()
	t.coolant_temp = 40
	local cooling = EngineThermal.newCoolingSystem()
	local lube = EngineThermal.newLubricationSystem()
	local damage = EngineThermal.newDamage()
	local stress = EngineThermal.newStressAccumulators()
	-- Engine OFF: 0 RPM
	EngineThermal.updateFull(t, cooling, lube, damage, stress, 0, 7000, 0, 0, 0, 1, 1/60)
	check(t.coolant_temp < 40, "coolant should cool toward ambient when engine OFF")
end)

print("\n=== Lubrication ===")

test("Oil pressure from pump at idle", function()
	local lube = EngineThermal.newLubricationSystem()
	EngineThermal.updateLubrication(lube, 0, 1/60, 800)
	check(lube.oil_pressure > 0, "should have pump pressure at 800 RPM")
	check(lube.oil_pressure <= lube.pump_pressure, "should not exceed pump max")
end)

test("Oil pressure zero when engine off", function()
	local lube = EngineThermal.newLubricationSystem()
	lube.oil_pressure = 300
	EngineThermal.updateLubrication(lube, 0, 1/60, 0)
	check(lube.oil_pressure < 300, "pressure should decay without pump")
end)

-- ============================================================
-- Damage
-- ============================================================

print("\n=== Damage ===")

test("Overheat increments cycles", function()
	local d = EngineThermal.newDamage()
	EngineThermal.checkOverheat(d, 115)
	check(d.overheating_cycles == 1, "should count one overheat cycle")
end)

test("Overrev increments cycles", function()
	local d = EngineThermal.newDamage()
	EngineThermal.checkOverrev(d, 8000, 7000)
	check(d.overrev_cycles == 1, "should count one overrev cycle")
end)

test("Oil starvation damages bearings", function()
	local d = EngineThermal.newDamage()
	EngineThermal.checkOilStarvation(d, 40)
	check(d.bearing_damage > 0, "bearing should accumulate damage")
end)

test("Severe oil starvation seizes engine", function()
	local d = EngineThermal.newDamage()
	for _ = 1, 200 do
		EngineThermal.checkOilStarvation(d, 5)
	end
	check(d.is_seized, "engine should seize from prolonged starvation")
end)

-- ============================================================
-- Stress accumulators
-- ============================================================

print("\n=== Stress ===")

test("Thermal stress accumulates above limit", function()
	local s = EngineThermal.newStressAccumulators()
	EngineThermal.accumulateThermal(s, 120, 110, 1)
	check(s.thermal_stress > 0, "should accumulate thermal stress")
end)

test("Thermal stress decays below limit", function()
	local s = EngineThermal.newStressAccumulators()
	s.thermal_stress = 0.5
	EngineThermal.accumulateThermal(s, 90, 110, 1)
	check(s.thermal_stress < 0.5, "should decay thermal stress when cool")
end)

test("Overrev stress accumulates", function()
	local s = EngineThermal.newStressAccumulators()
	EngineThermal.accumulateOverrev(s, 8000, 7000, 1)
	check(s.overrev_stress > 0, "should accumulate overrev stress")
end)

test("Lugging stress accumulates", function()
	local s = EngineThermal.newStressAccumulators()
	EngineThermal.accumulateLugging(s, 1200, 0.8, 4, 1)
	check(s.lugging_stress > 0, "should accumulate lugging stress")
end)

test("Blowup stage escalation", function()
	local s = EngineThermal.newStressAccumulators()
	check(EngineStressBlowupStage(s) == 0, "should start at Ok")
	s.thermal_stress = 0.3
	check(EngineStressBlowupStage(s) == 1, "should be WarningLamp at 0.3")
	s.thermal_stress = 0.6
	check(EngineStressBlowupStage(s) == 2, "should be TorqueDerate at 0.6")
	s.thermal_stress = 2.0
	check(EngineStressBlowupStage(s) == 6, "should be CompleteFailure at 2.0")
end)

test("Derate factor scales with stress", function()
	local s = EngineThermal.newStressAccumulators()
	approx(EngineStressDerateFactor(s), 1.0)
	s.thermal_stress = 1.0
	check(EngineStressDerateFactor(s) < 1.0, "should derate under stress")
end)

-- ============================================================
-- Combined update returns valid telemetry
-- ============================================================

print("\n=== Combined Update ===")

test("Full update returns all telemetry fields", function()
	local et = EngineThermal.new()
	local result = EngineThermal.updateFull(
		et.thermal, et.cooling, et.lubrication, et.damage, et.stress,
		800, 7000, 0, 0, 0, 1, 1/60
	)
	check(type(result.coolant_temp) == "number", "has coolant_temp")
	check(type(result.oil_temp) == "number", "has oil_temp")
	check(type(result.oil_pressure) == "number", "has oil_pressure")
	check(type(result.transmission_temp) == "number", "has transmission_temp")
	check(type(result.derate_factor) == "number", "has derate_factor")
	check(type(result.has_warning) == "boolean", "has has_warning")
	check(type(result.is_seized) == "boolean", "has is_seized")
	check(type(result.is_on_fire) == "boolean", "has is_on_fire")
	check(type(result.blowup_stage) == "number", "has blowup_stage")
	check(type(result.lugging_stress) == "number", "has lugging_stress")
	check(type(result.overrev_stress) == "number", "has overrev_stress")
	-- Verify values are reasonable
	check(result.coolant_temp >= 15, "coolant should be near ambient")
	check(result.oil_pressure >= 0, "oil pressure should be non-negative")
	check(result.derate_factor >= 0 and result.derate_factor <= 1, "derate in [0,1]")
end)

-- ============================================================
-- Summary
-- ============================================================

print(string.format("\n=== Results: %d/%d passed ===", passed, total))
if failed > 0 then
	process.exit(1)
end
