--[[
	Suspension system — raycast wheel, steering, Ackermann, fuel.
	Ported from crates/kemudi-engine/src/suspension.rs.
	Pure Luau, no Roblox API dependencies.
--]]

local Math = if script then require(script.Parent.Parent.math) else require("../math")

local Suspension = {}

-- ============================================================
-- Types (as factory functions)
-- ============================================================

local DEFAULT_WHEEL = {
	rest_length = 0.3,
	travel = 0.2,
	spring_rate = 30000.0,
	damping = 4000.0,
	rebound_damping = 3000.0,
	unsprung_mass = 15.0,
	tire_radius = 0.3,
}

function Suspension.newWheelConfig(overrides)
	local cfg = {}
	for k, v in DEFAULT_WHEEL do cfg[k] = v end
	if overrides then
		for k, v in overrides do cfg[k] = v end
	end
	return cfg
end

function Suspension.newConfig(overrides)
	local cfg = {
		wheels = {},
		anti_roll_bar_stiffness = 10000.0,
		bump_stop_rate = 500000.0,
	}
	if overrides then
		for k, v in overrides do cfg[k] = v end
	end
	if #cfg.wheels == 0 then
		for _ = 1, 4 do
			table.insert(cfg.wheels, Suspension.newWheelConfig())
		end
	end
	return cfg
end

function Suspension.newWheelState()
	return {
		compression = 0.0,
		load = 0.0,
		is_airborne = true,
		angular_speed = 0.0,
		contact_point = { 0, 0, 0 },
		suspension_force = 0.0,
		previous_suspension_length = nil,
	}
end

function Suspension.newSteeringConfig(overrides)
	local cfg = {
		steering_ratio = 15.0,
		steering_speed = 3.0,
		max_steering_angle = 0.5,
		speed_sensitivity = 0.04,
		wheelbase = 2.5,
		track_width = 1.6,
	}
	if overrides then
		for k, v in overrides do cfg[k] = v end
	end
	return cfg
end

-- ============================================================
-- Fuel system
-- ============================================================

function Suspension.newFuelTank(overrides)
	local tank = {
		current_level = 40.0,
		max_capacity = 60.0,
		base_consumption_rate = 0.004,
		idle_consumption_rate = 0.001,
		fuel_density = 0.75,
	}
	if overrides then
		for k, v in overrides do tank[k] = v end
	end
	return tank
end

function Suspension.updateFuel(tank, rpm, throttle, redline_rpm, dt)
	local load_fraction = math.clamp(rpm / math.max(redline_rpm, 1), 0, 1.5)
	local consumption
	if throttle > 0.01 then
		consumption = tank.base_consumption_rate * throttle * load_fraction * dt
	else
		consumption = tank.idle_consumption_rate * dt
	end
	tank.current_level = math.max(tank.current_level - consumption, 0)
	return {
		level = tank.current_level,
		percent = tank.current_level / math.max(tank.max_capacity, 1),
		fuel_mass = tank.current_level * tank.fuel_density,
	}
end

function Suspension.refuel(tank, amount)
	tank.current_level = math.clamp(tank.current_level + amount, 0, tank.max_capacity)
end

-- ============================================================
-- Core suspension
-- ============================================================

-- Damper velocity from geometry (current vs previous suspension length).
function Suspension.suspensionLengthVelocity(current_length, previous_length, dt)
	if previous_length == nil or dt <= 0 then return 0 end
	return (current_length - previous_length) / math.max(dt, 1e-6)
end

-- Progressive bump stop: quadratic engagement past 88% compression.
function Suspension.progressiveBumpStopForce(compression, travel, bump_stop_rate)
	if compression <= 0.88 then return 0 end
	local engagement = math.clamp((compression - 0.88) / 0.12, 0, 1)
	return bump_stop_rate * travel * engagement * engagement
end

-- Raycast wheel — the core suspension force computation.
-- Returns: { force, contact_point, normal }, compression, is_airborne
function Suspension.raycastWheel(config, wheel_pos, susp_vel, terrain_height)
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

	-- Spring (Hooke's law)
	local spring_force = config.spring_rate * compression * travel

	-- Damper (asymmetric compression/rebound)
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

-- Raycast wheel with explicit surface normal.
function Suspension.raycastWheelWithNormal(config, wheel_pos, susp_vel, terrain_height, surface_normal)
	local result, compression, airborne = Suspension.raycastWheel(config, wheel_pos, susp_vel, terrain_height)
	-- Normalize surface normal
	local len = math.sqrt(surface_normal[1]^2 + surface_normal[2]^2 + surface_normal[3]^2)
	if len > 1e-8 then
		result.normal = { surface_normal[1]/len, surface_normal[2]/len, surface_normal[3]/len }
	end
	return result, compression, airborne
end

-- ============================================================
-- Steering
-- ============================================================

-- Speed-sensitive steering with rate limiting.
function Suspension.updateSteering(config, input_angle, vehicle_speed, current_angle, dt)
	local speed_factor = 1 / (1 + vehicle_speed * config.speed_sensitivity)
	local max_angle = config.max_steering_angle * speed_factor
	local target = math.clamp(input_angle, -max_angle, max_angle)
	local max_change = config.steering_speed * dt
	local diff = target - current_angle
	if math.abs(diff) <= max_change then
		return math.clamp(target, -max_angle, max_angle)
	end
	return math.clamp(current_angle + math.sign(diff) * max_change, -max_angle, max_angle)
end

-- Ackermann geometry: split virtual center-wheel angle into left/right.
function Suspension.ackermannAngles(steering_angle, wheelbase, track_width)
	local half_track = track_width / 2
	local sign = Math.sign(steering_angle)
	local abs_angle = math.min(math.abs(steering_angle), math.pi / 2 - 0.001)

	if abs_angle < 1e-6 then
		return 0, 0
	end

	local center_radius = wheelbase / math.tan(abs_angle)
	local inner = sign * math.atan(wheelbase / math.max(center_radius - half_track, 0.05))
	local outer = sign * math.atan(wheelbase / (center_radius + half_track))

	-- Positive input = right turn → right wheel is inner
	if steering_angle > 0 then
		return outer, inner
	else
		return inner, outer
	end
end

return Suspension
