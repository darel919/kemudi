--[[
	Wheel angular momentum model — torque integration, slip, lockup.
	Per-wheel rotational dynamics.
	Pure Luau, no Roblox API dependencies.
--]]

local WheelDynamics = {}

-- ============================================================
-- NaN / infinity guard
-- ============================================================
local function safeFinite(v)
	if v == nil then return 0 end
	if v ~= v then return 0 end -- NaN
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

-- ============================================================
-- Factory functions
-- ============================================================

function WheelDynamics.newState(overrides)
	local state = {
		angular_velocity = 0.0,    -- rad/s (positive = forward rotation)
		inertia = 0.0,             -- kg·m² (derived from mass and radius)
		brake_torque = 0.0,        -- N·m applied this frame
		drive_torque = 0.0,        -- N·m from drivetrain
		tire_contact_force = 0.0,  -- N from tire contact patch
		tire_radius = 0.0,         -- m; populated from the vehicle asset
		is_locked = false,          -- handbrake / lockup
		slip_ratio = 0.0,          -- current longitudinal slip ratio
		slip_angle = 0.0,          -- current lateral slip angle (rad)
		angular_energy = 0.0,       -- 0.5 * I * ω² for telemetry
	}
	if overrides then for k, v in overrides do state[k] = v end end
	return state
end

function WheelDynamics.newConfig(overrides)
	assert(type(overrides) == "table", "wheel dynamics parameters must be authored by the vehicle asset")
	assert(type(overrides.wheel_mass) == "number" and overrides.wheel_mass > 0, "wheel_mass must be positive")
	assert(type(overrides.tire_radius) == "number" and overrides.tire_radius > 0, "tire_radius must be positive")
	return overrides
end

-- ============================================================
-- Inertia
-- ============================================================

function WheelDynamics.computeInertia(config)
	local mass = safeFinite(config.wheel_mass)
	local radius = safeFinite(config.tire_radius)
	return 0.5 * mass * radius * radius
end

-- ============================================================
-- Core integration
-- ============================================================

function WheelDynamics.integrate(state, config, ground_speed_mps, dt)
	-- Recompute inertia from config in case mass/radius changed
	state.inertia = safeFinite(WheelDynamics.computeInertia(config))
	state.tire_radius = safeFinite(config.tire_radius)

	-- NaN-guard inputs
	state.drive_torque = safeFinite(state.drive_torque)
	state.brake_torque = safeFinite(state.brake_torque)
	state.tire_contact_force = safeFinite(state.tire_contact_force)
	ground_speed_mps = safeFinite(ground_speed_mps or 0)
	dt = safeFinite(dt)

	if dt <= 0 then return end

	-- Locked wheel: clamp to zero, skip integration
	if state.is_locked then
		state.angular_velocity = 0.0
		local abs_g = math.abs(ground_speed_mps)
		if abs_g > 0.5 then
			state.slip_ratio = clamp(-ground_speed_mps / abs_g, -2, 2)
		else
			state.slip_ratio = 0.0
		end
		state.angular_energy = 0.0
		return
	end

	-- Brake torque opposes wheel rotation. Applying a positive brake torque
	-- unconditionally makes a stationary wheel spin backward until the safety
	-- clamp (the runtime symptom appears as exactly -200 rad/s on every wheel).
	local brake_torque = 0.0
	if math.abs(state.angular_velocity) > 1e-8 then
		brake_torque = math.sign(state.angular_velocity) * math.abs(state.brake_torque)
	elseif math.abs(state.drive_torque) > 1e-8 then
		brake_torque = math.sign(state.drive_torque) * math.abs(state.brake_torque)
	end

	-- Net torque: τ_net = drive - opposing brake - contact * radius
	local net_torque = state.drive_torque - brake_torque
		- state.tire_contact_force * state.tire_radius
	net_torque = safeFinite(net_torque)

	-- Angular acceleration: α = τ / I
	local inv_inertia = if state.inertia > 1e-8 then 1.0 / state.inertia else 0.0
	local alpha = safeFinite(net_torque * inv_inertia)

	-- Euler integration
	state.angular_velocity = safeFinite(state.angular_velocity + alpha * dt)

	-- Clamp to prevent explosion: ±200 rad/s (~1900 RPM)
	state.angular_velocity = clamp(state.angular_velocity, -200.0, 200.0)

	-- Slip ratio: (wheel_surface_speed - ground_speed) / max(|ground_speed|, 0.5)
	local wheel_surface_speed = state.angular_velocity * state.tire_radius
	local abs_ground = math.max(math.abs(ground_speed_mps), 0.5)
	state.slip_ratio = clamp(
		(wheel_surface_speed - ground_speed_mps) / abs_ground,
		-2.0, 2.0
	)
	state.slip_ratio = safeFinite(state.slip_ratio)

	-- Rotational kinetic energy for telemetry
	state.angular_energy = safeFinite(0.5 * state.inertia * state.angular_velocity * state.angular_velocity)
end

-- ============================================================
-- Accessors
-- ============================================================

function WheelDynamics.getSlipRatio(state)
	return state.slip_ratio
end

function WheelDynamics.getWheelSpeed(state)
	return state.angular_velocity * state.tire_radius
end

-- ============================================================
-- Telemetry
-- ============================================================

function WheelDynamics.telemetry(state)
	return {
		angular_velocity = state.angular_velocity,
		slip_ratio = state.slip_ratio,
		angular_energy = state.angular_energy,
		is_locked = state.is_locked,
	}
end

return WheelDynamics
