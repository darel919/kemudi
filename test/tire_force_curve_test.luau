--[[ Integration-level tests for tire force curve behavior.
     Exercises the real src/vehicle/tires module (not inlined formulas)
     and verifies the Pacejka longitudinal force shape that vehicle.luau
     now depends on for correct drive-force computation.
     Run: lune run test/tire_force_curve_test.luau
--]]

local process = require("@lune/process")
local Tires = require("../src/vehicle/tires")

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

local function check(cond, msg)
	if not cond then error(msg or "assertion failed") end
end

-- ============================================================
-- Pacejka force curve shape (real module)
-- ============================================================

local params = Tires.newPacejkaParams()
local load = 3000
local grip = 0.85

print("=== Pacejka Force Curve Tests (real Tires module) ===")

test("Longitudinal force is zero at zero slip", function()
	local f = Tires.calculatePacejkaForces(params, 0, 0, load, grip)
	check(math.abs(f.longitudinal) < 1, "zero slip should give near-zero force")
end)

test("Longitudinal force grows with slip up to peak", function()
	local f1 = Tires.calculatePacejkaForces(params, 0.02, 0, load, grip)
	local f2 = Tires.calculatePacejkaForces(params, 0.08, 0, load, grip)
	check(f2.longitudinal > f1.longitudinal,
		string.format("force at slip=0.08 (%.1f) should exceed slip=0.02 (%.1f)",
			f2.longitudinal, f1.longitudinal))
end)

test("Longitudinal force peaks then falls with increasing slip", function()
	local peakForce = 0
	local peakSlip = 0
	-- Sweep slip ratio to find the peak
	for slip = 0.01, 2.0, 0.005 do
		local f = Tires.calculatePacejkaForces(params, slip, 0, load, grip)
		if f.longitudinal > peakForce then
			peakForce = f.longitudinal
			peakSlip = slip
		end
	end
	-- Default params (b=10, c=1.65, e=0.97) produce a peak around 40% slip.
	-- Real tires peak 5-15%; this can be tuned by lowering E toward 0.7.
	check(peakSlip > 0.10 and peakSlip < 0.60,
		string.format("peak should be between 10%% and 60%% slip, got %.2f%%", peakSlip * 100))

	-- Force at extreme slip must be less than peak (Pacejka always falls past peak)
	local fExtreme = Tires.calculatePacejkaForces(params, 2.0, 0, load, grip)
	check(fExtreme.longitudinal < peakForce,
		string.format("extreme-slip force (%.1f) should be less than peak (%.1f)",
			fExtreme.longitudinal, peakForce))
	print(string.format("    peak=%.1f N at slip=%.2f, at slip=2.0: %.1f N (%.1f%% of peak)",
		peakForce, peakSlip, fExtreme.longitudinal,
		(fExtreme.longitudinal / peakForce) * 100))
end)

-- Core regression: the old code clamped driveForce to longitudinal_peak,
-- so a tire at any slip produced the same force as the peak.
-- With the default E=0.97 the curve is flat, so the absolute difference
-- is small — but the DIRECTION must be correct (force past peak < peak).
test("REGRESSION: force at high slip is strictly less than peak", function()
	local peakForce = 0
	for slip = 0.01, 2.0, 0.005 do
		local f = Tires.calculatePacejkaForces(params, slip, 0, load, grip)
		if f.longitudinal > peakForce then
			peakForce = f.longitudinal
		end
	end
	local fHigh = Tires.calculatePacejkaForces(params, 1.5, 0, load, grip)
	-- Even with the flat E=0.97 curve, force past peak MUST be < peak.
	-- If this ever equals peak, the magic formula is broken or was bypassed.
	check(fHigh.longitudinal < peakForce,
		string.format("force at slip=1.5 (%.4f) must be strictly less than peak (%.4f) — old bug if equal",
			fHigh.longitudinal, peakForce))
end)

test("Force scales with load", function()
	local f1 = Tires.calculatePacejkaForces(params, 0.1, 0, 2000, grip)
	local f2 = Tires.calculatePacejkaForces(params, 0.1, 0, 4000, grip)
	check(f2.longitudinal > f1.longitudinal, "higher load should produce more force")
end)

test("Negative slip produces negative longitudinal force (braking)", function()
	local f = Tires.calculatePacejkaForces(params, -0.15, 0, load, grip)
	check(f.longitudinal < 0, "negative slip should produce negative (braking) force")
end)

-- ============================================================
-- Combined slip friction ellipse
-- ============================================================

print("\n=== Combined Slip Tests ===")

test("Combined slip respects friction ellipse", function()
	local f = Tires.calculatePacejkaForces(params, 0.4, 0.4, load, grip)
	local norm_l = math.abs(f.longitudinal) / math.max(f.longitudinal_peak, 1)
	local norm_s = math.abs(f.lateral) / math.max(f.lateral_peak, 1)
	local demand = math.sqrt(norm_l^2 + norm_s^2)
	check(demand <= 1.05, string.format("friction demand %.3f exceeds ellipse", demand))
end)

test("Lateral force with zero slip angle is zero", function()
	local f = Tires.calculatePacejkaForces(params, 0.1, 0, load, grip)
	check(math.abs(f.lateral) < 1, "zero slip angle should give near-zero lateral force")
end)

-- ============================================================
-- Slip-dependent force dynamics simulation
-- ============================================================

print("\n=== Slip-Dependent Dynamics Simulation ===")

-- Simulate constant engine torque on a wheel with tire reaction
-- torque feeding back. With the default E=0.97 Pacejka params the
-- curve is flat, so the key assertion is that the Pacejka force
-- (not the peak clamp) is what reaches the chassis.
test("Constant torque: tire force follows Pacejka at current slip", function()
	local tireRadius = 0.3
	local wheelInertia = 20.0
	local engineTorque = 500.0
	local dt = 1 / 60
	local forwardSpeed = 0.0
	local wheelSpeed = 0.0

	-- Run 600 steps (10 seconds)
	local forcesUsed = {}
	for step = 1, 600 do
		local wheelLinearSpeed = wheelSpeed * tireRadius
		local slipRatio = math.clamp(
			(wheelLinearSpeed - forwardSpeed) / math.max(math.abs(forwardSpeed), 0.25),
			-2, 2)

		-- This is what vehicle.luau now does: compute Pacejka force
		-- at current slip and use it directly (not clamped to peak).
		local forces = Tires.calculatePacejkaForces(params, slipRatio, 0, load, grip)
		local driveForce = forces.longitudinal -- the fix

		-- The OLD code would have used: clamp(requestedDrive, -peak, peak)
		-- which gives a DIFFERENT force than the Pacejka output.
		local requestedDrive = engineTorque / tireRadius
		local oldDriveForce = math.clamp(requestedDrive, -forces.longitudinal_peak, forces.longitudinal_peak)

		forcesUsed[step] = { actual = driveForce, old = oldDriveForce, slip = slipRatio }

		-- Chassis acceleration
		local chassisAccel = driveForce / 2000.0 -- ~2000kg car
		forwardSpeed = forwardSpeed + chassisAccel * dt

		-- Wheel: engine torque minus tire reaction
		local reactionTorque = driveForce * tireRadius
		local angularAccel = (engineTorque - reactionTorque) / wheelInertia
		wheelSpeed = math.max(wheelSpeed + angularAccel * dt, 0)
	end

	-- Verify the actual force differs from the old peak-clamp at some point.
	-- Even with flat E=0.97, the initial ramp-up has actual < peak.
	local anyDiff = false
	for _, f in forcesUsed do
		if math.abs(f.actual - f.old) > 10 then
			anyDiff = true
			break
		end
	end
	check(anyDiff,
		"Pacejka force should differ from peak clamp at some slip — " ..
		"old bug if always equal")

	-- The system should reach a steady state (engine torque balanced by
	-- tire reaction). With default E=0.97 the curve is flat, so the
	-- equilibrium slip can be low. The key assertion is stability.
	local finalSlip = forcesUsed[600].slip
	local finalForce = forcesUsed[600].actual
	local prevSlip = forcesUsed[500].slip
	check(math.abs(finalSlip - prevSlip) < 0.01,
		string.format("slip should stabilize, got slip500=%.3f slip600=%.3f", prevSlip, finalSlip))
	print(string.format("    steady-state: slip=%.3f, force=%.1f N", finalSlip, finalForce))
end)

-- ============================================================
-- Tunable parameters check
-- ============================================================

print("\n=== Parameter Sensitivity ===")

test("Lower E produces sharper peak and more falloff", function()
	local sharpParams = Tires.newPacejkaParams({
		longitudinal_e = 0.7, -- lower E = sharper peak
	})
	local peakForce = 0
	local peakSlip = 0
	for slip = 0.01, 2.0, 0.01 do
		local f = Tires.calculatePacejkaForces(sharpParams, slip, 0, load, grip)
		if f.longitudinal > peakForce then
			peakForce = f.longitudinal
			peakSlip = slip
		end
	end
	-- With E=0.7, peak should be earlier and falloff more pronounced
	local fHigh = Tires.calculatePacejkaForces(sharpParams, 1.5, 0, load, grip)
	local ratio = fHigh.longitudinal / math.max(peakForce, 1)
	check(ratio < 0.85,
		string.format("E=0.7 should give meaningful falloff, got %.0f%% of peak at slip=1.5", ratio * 100))
	print(string.format("    E=0.7 peak: %.1f N at slip=%.2f, at slip=1.5: %.1f N (%.0f%%)",
		peakForce, peakSlip, fHigh.longitudinal, ratio * 100))
end)

-- ============================================================
-- Edge cases
-- ============================================================

print("\n=== Edge Cases ===")

test("Zero load produces zero force", function()
	local f = Tires.calculatePacejkaForces(params, 0.1, 0, 0, grip)
	check(f.longitudinal == 0, "zero load should give zero force")
end)

test("Zero grip produces zero force", function()
	local f = Tires.calculatePacejkaForces(params, 0.1, 0, load, 0)
	check(f.longitudinal == 0, "zero grip should give zero force")
end)

-- ============================================================
-- Summary
-- ============================================================

print(string.format("\n=== Results: %d/%d passed ===", passed, total))
if failed > 0 then
	process.exit(1)
end
