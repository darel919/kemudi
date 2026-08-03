--[[
	Vehicle system unit tests for Lune.
	Tests core formulas from suspension, tires, drivetrain, safety.
	Run: lune run test/vehicle_unit_test.luau
--]]

local process = require("@lune/process")

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
		error(string.format("expected %.10f, got %.10f (diff %.10f)", b, a, math.abs(a - b)))
	end
end

local function check(cond, msg)
	if not cond then error(msg or "assertion failed") end
end

-- ============================================================
-- Math utilities (inlined from src/math/init.luau)
-- ============================================================

local function finite_or_zero(v)
	if v ~= v then return 0 end
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

local function sign(v)
	if v > 0 then return 1 end
	if v < 0 then return -1 end
	return 0
end

-- ============================================================
-- Suspension formulas (inlined from src/vehicle/suspension.luau)
-- ============================================================

local function raycast_wheel(config, wheel_pos, susp_vel, terrain_height)
	local mount_y = wheel_pos[2]
	local tire_radius = config.tire_radius
	local rest_length = config.rest_length
	local travel = config.travel

	local suspension_length = mount_y - terrain_height - tire_radius
	local max_extension = rest_length + travel + 0.01

	if suspension_length > max_extension then
		return { force = 0, contact_point = { wheel_pos[1], terrain_height, wheel_pos[3] }, normal = { 0, 1, 0 } }, 0, true
	end

	local compression = math.clamp((rest_length - suspension_length) / math.max(travel, 1e-6), 0, 1)
	local spring_force = config.spring_rate * compression * travel

	local damping_force
	if susp_vel < 0 then
		damping_force = config.damping * math.abs(susp_vel)
	else
		damping_force = -config.rebound_damping * susp_vel
	end

	local total_force = math.max(spring_force + damping_force, 0)
	local contact = { wheel_pos[1], terrain_height, wheel_pos[3] }
	return { force = total_force, contact_point = contact, normal = { 0, 1, 0 } }, compression, false
end

local function ackermann_angles(steering_angle, wheelbase, track_width)
	local half_track = track_width / 2
	local s = sign(steering_angle)
	local abs_angle = math.min(math.abs(steering_angle), math.pi / 2 - 0.001)

	if abs_angle < 1e-6 then return 0, 0 end

	local center_radius = wheelbase / math.tan(abs_angle)
	local inner = s * math.atan(wheelbase / math.max(center_radius - half_track, 0.05))
	local outer = s * math.atan(wheelbase / (center_radius + half_track))

	if steering_angle > 0 then
		return outer, inner
	else
		return inner, outer
	end
end

-- ============================================================
-- Pacejka magic formula (inlined from src/vehicle/tires.luau)
-- ============================================================

local function magic_formula(slip, b, c, d, e)
	local s = math.clamp(slip, -4, 4)
	local sb = math.max(b, 0)
	local sc = math.max(c, 0)
	local sd = math.max(d, 0)
	local se = math.clamp(e, -1, 1)
	local bx = sb * s
	local val = sd * math.sin(sc * math.atan(bx - se * (bx - math.atan(bx))))
	if val ~= val then return 0 end
	if val == math.huge or val == -math.huge then return 0 end
	return val
end

local function pacejka_forces(params, slip_ratio, slip_angle, wheel_load, grip)
	if wheel_load <= 0 or grip <= 0 then
		return { longitudinal = 0, lateral = 0, longitudinal_peak = 0, lateral_peak = 0 }
	end

	local normalized_load = wheel_load / 4000
	local load_factor = math.clamp(1.0 - 0.18 * (normalized_load - 1.0), 0.65, 1.15)

	local long_peak = wheel_load * grip * load_factor * params.longitudinal_d
	local lat_peak = wheel_load * grip * load_factor * params.lateral_d

	local long_force = long_peak * magic_formula(slip_ratio, params.longitudinal_b, params.longitudinal_c, 1.0, params.longitudinal_e)
	local lat_force = -(lat_peak * magic_formula(slip_angle, params.lateral_b, params.lateral_c, 1.0, params.lateral_e))

	local norm_l = if long_peak > 0 then math.abs(long_force) / long_peak else 0
	local norm_s = if lat_peak > 0 then math.abs(lat_force) / lat_peak else 0
	local demand = math.sqrt(norm_l * norm_l + norm_s * norm_s)
	if demand > 1.0 then
		long_force = long_force / demand
		lat_force = lat_force / demand
	end

	return {
		longitudinal = long_force, lateral = lat_force,
		longitudinal_peak = long_peak, lateral_peak = lat_peak,
	}
end

-- ============================================================
-- Drivetrain formulas (inlined from src/vehicle/drivetrain.luau)
-- ============================================================

local function engine_torque_at_rpm(rpm)
	local curve = {
		{ 0, 100 }, { 1000, 150 }, { 2000, 200 }, { 3000, 250 },
		{ 4000, 280 }, { 5000, 260 }, { 6000, 220 }, { 7000, 160 }, { 8000, 100 },
	}
	if rpm <= curve[1][1] then return curve[1][2] end
	if rpm >= curve[#curve][1] then return curve[#curve][2] end
	for i = 1, #curve - 1 do
		local r0, t0 = curve[i][1], curve[i][2]
		local r1, t1 = curve[i+1][1], curve[i+1][2]
		if rpm >= r0 and rpm <= r1 then
			return t0 + (rpm - r0) / (r1 - r0) * (t1 - t0)
		end
	end
	return 0
end

local function total_ratio(gear_ratios, final_drive, current_gear)
	if current_gear >= 1 and current_gear <= #gear_ratios then
		return gear_ratios[current_gear] * final_drive
	elseif current_gear == -1 then
		return -3.2 * final_drive
	end
	return 0
end

-- ============================================================
-- Safety formulas (inlined from src/vehicle/safety.luau)
-- ============================================================

local function abs_update(enabled, slip_threshold, wheel_speeds, vehicle_speed, brake_command)
	if not enabled then
		return { brake_command, brake_command, brake_command, brake_command }, false
	end

	local pressures = {}
	local any_active = false
	for i = 1, 4 do
		if vehicle_speed > 0.5 then
			local slip = 1.0 - (wheel_speeds[i] / vehicle_speed)
			slip = math.clamp(slip, 0, 1.5)
			if slip > slip_threshold and brake_command > 0.1 then
				local reduction = math.clamp((slip - slip_threshold) / slip_threshold, 0, 1)
				pressures[i] = brake_command * (1 - reduction * 0.7)
				any_active = true
			else
				pressures[i] = brake_command
			end
		else
			pressures[i] = brake_command
		end
	end
	return pressures, any_active
end

local function tc_update(enabled, threshold, wheel_speeds, vehicle_speed, driven_wheels, throttle)
	if not enabled or vehicle_speed <= 0.5 then return 1.0 end

	local modifier = 1.0
	for _, idx in driven_wheels do
		if idx >= 1 and idx <= 4 then
			local slip = math.max(wheel_speeds[idx] / vehicle_speed - 1.0, 0)
			if slip > threshold and throttle > 0.1 then
				local excess = (slip - threshold) / threshold
				local reduction = math.min(excess * 0.5, 0.5)
				modifier = math.max(1.0 - reduction, 0.3)
			end
		end
	end
	return modifier
end

-- ============================================================
-- Tests
-- ============================================================

print("=== Suspension Tests ===")

test("Raycast wheel airborne", function()
	local cfg = { rest_length = 0.3, travel = 0.2, spring_rate = 30000, damping = 4000, rebound_damping = 3000, tire_radius = 0.3 }
	local f, comp, air = raycast_wheel(cfg, {0, 2.0, 0}, 0, 0)
	check(air, "should be airborne")
	approx(f.force, 0)
end)

test("Raycast wheel grounded", function()
	local cfg = { rest_length = 0.3, travel = 0.2, spring_rate = 30000, damping = 4000, rebound_damping = 3000, tire_radius = 0.3 }
	local f, comp, air = raycast_wheel(cfg, {0, 0.5, 0}, 0, 0)
	check(not air, "should be grounded")
	check(f.force > 0, "should have positive force")
	-- suspension_length = 0.5 - 0 - 0.3 = 0.2; compression = (0.3 - 0.2) / 0.2 = 0.5
	approx(comp, 0.5, 0.01)
end)

test("Ackermann at zero steering", function()
	local l, r = ackermann_angles(0, 2.5, 1.6)
	approx(l, 0)
	approx(r, 0)
end)

test("Ackermann inner > outer", function()
	local l, r = ackermann_angles(0.3, 2.5, 1.6)
	check(math.abs(r) > math.abs(l), "right wheel should turn more for positive angle")
end)

test("Ackermann symmetry", function()
	local l1, r1 = ackermann_angles(0.3, 2.5, 1.6)
	local l2, r2 = ackermann_angles(-0.3, 2.5, 1.6)
	-- Positive steer: (outer, inner). Negative steer: (inner, outer).
	-- l1=outer, r1=inner for right turn. l2=inner, r2=outer for left turn.
	-- Symmetry: |outer_right| ≈ |outer_left|, |inner_right| ≈ |inner_left|
	approx(math.abs(l1), math.abs(r2), 0.01) -- outer magnitudes match
	approx(math.abs(r1), math.abs(l2), 0.01) -- inner magnitudes match
end)

print("\n=== Pacejka Tests ===")

local DEFAULT_PARAMS = {
	longitudinal_b = 10, longitudinal_c = 1.65, longitudinal_d = 1.0, longitudinal_e = 0.97,
	lateral_b = 10, lateral_c = 1.9, lateral_d = 1.0, lateral_e = 0.97,
}

test("Pacejka zero slip → zero force", function()
	local r = pacejka_forces(DEFAULT_PARAMS, 0, 0, 3000, 0.85)
	approx(r.longitudinal, 0, 0.01)
	approx(r.lateral, 0, 0.01)
end)

test("Pacejka positive slip → positive longitudinal", function()
	local r = pacejka_forces(DEFAULT_PARAMS, 0.1, 0, 3000, 0.85)
	check(r.longitudinal > 0, "positive slip should produce positive force")
end)

test("Pacejka slip angle → lateral force", function()
	local r = pacejka_forces(DEFAULT_PARAMS, 0, 0.1, 3000, 0.85)
	check(r.lateral < 0, "slip angle should produce negative lateral (left)")
end)

test("Pacejka peaks scale with load", function()
	local r1 = pacejka_forces(DEFAULT_PARAMS, 0, 0, 2000, 0.85)
	local r2 = pacejka_forces(DEFAULT_PARAMS, 0, 0, 4000, 0.85)
	check(r2.lateral_peak > r1.lateral_peak, "higher load should give higher peak")
end)

test("Pacejka combined slip ≤ 1.0 friction ellipse", function()
	local r = pacejka_forces(DEFAULT_PARAMS, 0.5, 0.5, 3000, 0.85)
	local demand = math.sqrt((r.longitudinal / math.max(r.longitudinal_peak, 0.01))^2
		+ (r.lateral / math.max(r.lateral_peak, 0.01))^2)
	check(demand <= 1.05, "combined slip should respect friction ellipse")
end)

print("\n=== Drivetrain Tests ===")

test("Engine torque at peak RPM", function()
	approx(engine_torque_at_rpm(4000), 280)
end)

test("Engine torque at idle", function()
	approx(engine_torque_at_rpm(0), 100)
end)

test("Engine torque interpolation", function()
	local t = engine_torque_at_rpm(3500)
	check(t > 250 and t < 280, "should interpolate between 3000 and 4000")
end)

test("Total ratio gear 1", function()
	local r = total_ratio({3.5, 2.1, 1.4, 1.0, 0.7}, 3.7, 1)
	approx(r, 3.5 * 3.7)
end)

test("Total ratio reverse", function()
	local r = total_ratio({3.5, 2.1, 1.4, 1.0, 0.7}, 3.7, -1)
	approx(r, -3.2 * 3.7)
end)

test("Total ratio neutral", function()
	local r = total_ratio({3.5, 2.1, 1.4, 1.0, 0.7}, 3.7, 0)
	approx(r, 0)
end)

print("\n=== Safety Tests ===")

test("ABS no intervention at low slip", function()
	-- slip = 1 - (wheel_speed/vehicle_speed) = 1 - (28/30) = 0.067 < 0.15
	local p, active = abs_update(true, 0.15, {28, 28, 28, 28}, 30, 0.8)
	approx(p[1], 0.8)
	check(not active)
end)

test("ABS intervention at high slip", function()
	-- wheel[1] slip = 1 - (5/30) = 0.833 > 0.15, wheel[2] slip = 1 - (28/30) = 0.067 < 0.15
	local p, active = abs_update(true, 0.15, {5, 28, 28, 28}, 30, 0.8)
	check(active, "should be active")
	check(p[1] < 0.8, "should reduce brake on slipping wheel")
	check(p[2] == 0.8, "should not affect non-slipping wheel")
end)

test("ABS disabled passthrough", function()
	local p, active = abs_update(false, 0.15, {5, 5, 5, 5}, 30, 0.8)
	approx(p[1], 0.8)
	check(not active)
end)

test("TC no intervention at low slip", function()
	local m = tc_update(true, 0.12, {20, 20, 20, 20}, 30, {2, 3}, 0.5)
	approx(m, 1.0)
end)

test("TC intervention at high slip", function()
	local m = tc_update(true, 0.12, {40, 40, 30, 30}, 20, {2, 3}, 0.8)
	check(m < 1.0, "should reduce throttle")
	check(m >= 0.3, "should have floor")
end)

test("TC disabled passthrough", function()
	local m = tc_update(false, 0.12, {40, 40, 30, 30}, 20, {2, 3}, 0.8)
	approx(m, 1.0)
end)

-- ============================================================
-- Summary
-- ============================================================

print(string.format("\n=== Results: %d/%d passed ===", passed, total))
if failed > 0 then
	process.exit(1)
end
