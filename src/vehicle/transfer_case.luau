--[[
	Transfer Case — selectable 4WD modes (2H/4A/4H/4L/Neutral) and
	low-range gearing for off-road vehicle simulation.
	Pure Luau, no Roblox API dependencies.

	ALL CONFIG MUST BE PROVIDED BY VEHICLE ASSET.
	No vehicle-specific defaults in engine code.
--]]

local TransferCase = {}

-- ============================================================
-- Helpers
-- ============================================================

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

local function nan_safe(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

-- ============================================================
-- Config
-- ============================================================

function TransferCase.newConfig(overrides)
	overrides = overrides or {}
	return {
		-- Required: low-range gear ratio (e.g. 2.64:1). Set to 1.0 for no low range.
		low_range_ratio = nan_safe(overrides.low_range_ratio, 1.0),

		-- Center coupling stiffness for 4A mode (auto clutch engagement rate)
		center_coupling_rate = nan_safe(overrides.center_coupling_rate, 0.1),

		-- Front axle disconnect: if true, front is disconnected in 2H
		front_axle_disconnect = nan_safe(overrides.front_axle_disconnect, false),

		-- Rear differential configuration (e.g. "Open" | "Locked" | "E-Locker")
		rear_diff_type = overrides.rear_diff_type or "Open",
		-- E-locker max engagement speed in km/h
		rear_locker_max_speed = nan_safe(overrides.rear_locker_max_speed, 30),

		-- Front differential configuration
		front_diff_type = overrides.front_diff_type or "Open",

		-- Torque split ratio (front:rear) for locked modes (default 50:50)
		torque_split_front = nan_safe(overrides.torque_split_front, 0.5),

		-- Efficiency per mode (0 = no loss, 1 = full loss)
		efficiency_2h = nan_safe(overrides.efficiency_2h, 0.92),
		efficiency_4h = nan_safe(overrides.efficiency_4h, 0.88),
		efficiency_4l = nan_safe(overrides.efficiency_4l, 0.85),
	}
end

-- ============================================================
-- State
-- ============================================================

function TransferCase.newState()
	return {
		mode = "2H",  -- 2H | 4A | 4H | 4L | Neutral
		target_mode = nil,  -- pending mode change (transitional)
		center_coupling = 0.0,  -- 0 = disengaged, 1 = locked (4A auto-fills)
		rear_locker_engaged = false,
		rear_locker_requested = false,
		low_range_active = false,
		front_engaged = false,  -- true when front axle receives torque
		mode_transition_timer = 0,
	}
end

-- ============================================================
-- Mode selection
-- ============================================================

local VALID_MODES = { "2H", "4A", "4H", "4L", "Neutral" }

function TransferCase.requestMode(tc, mode, vehicle_speed_kmh, brake_applied)
	if not mode then return false end
	local found = false
	for _, m in VALID_MODES do
		if m == mode then found = true; break end
	end
	if not found then return false end
	if mode == tc.mode then return false end

	-- Safety gates
	local stopped = vehicle_speed_kmh < 3
	local stepping_brake = brake_applied and brake_applied > 0.1

	-- 4L requires stop + brake (or very low speed)
	if mode == "4L" and not stopped and not (vehicle_speed_kmh < 5 and stepping_brake) then
		return false -- Must be nearly stopped for low range engagement
	end

	-- Neutral: only with brake at low speed
	if mode == "Neutral" and vehicle_speed_kmh > 1 then
		return false
	end

	tc.target_mode = mode
	tc.mode_transition_timer = 0
	return true
end

-- ============================================================
-- Update (per-frame)
-- ============================================================

function TransferCase.update(tc, config, vehicle_speed_kmh, dt)
	dt = nan_safe(dt, 1/60)
	vehicle_speed_kmh = nan_safe(vehicle_speed_kmh, 0)

	-- Mode transitions (with mechanical latency)
	if tc.target_mode and tc.target_mode ~= tc.mode then
		tc.mode_transition_timer = (tc.mode_transition_timer or 0) + dt
		-- 0.2s transition for electric servo, 0.5s for mechanical
		local transition_time = if tc.target_mode == "4L" then 0.5 else 0.2
		if tc.mode_transition_timer >= transition_time then
			tc.mode = tc.target_mode
			tc.target_mode = nil
			tc.mode_transition_timer = 0
		end
	end

	-- Mode-specific center coupling and front engagement
	if tc.mode == "2H" then
		tc.center_coupling = 0.0
		tc.front_engaged = not config.front_axle_disconnect
		tc.low_range_active = false
	elseif tc.mode == "4A" then
		-- Auto: center coupling based on speed differential
		-- Simplifiest model: coupling increases with speed
		local coupling_rate = config.center_coupling_rate or 0.1
		tc.center_coupling = clamp(tc.center_coupling + coupling_rate * dt, 0, 0.95)
		tc.front_engaged = true
		tc.low_range_active = false
	elseif tc.mode == "4H" then
		tc.center_coupling = 1.0  -- locked
		tc.front_engaged = true
		tc.low_range_active = false
	elseif tc.mode == "4L" then
		tc.center_coupling = 1.0  -- locked in low range
		tc.front_engaged = true
		tc.low_range_active = true
	elseif tc.mode == "Neutral" then
		tc.center_coupling = 0.0
		tc.front_engaged = false
		tc.low_range_active = false
	end

	-- Rear electronic locker
	if tc.rear_locker_requested then
		if vehicle_speed_kmh < config.rear_locker_max_speed then
			tc.rear_locker_engaged = true
		else
			tc.rear_locker_engaged = false
		end
	else
		tc.rear_locker_engaged = false
	end
end

-- ============================================================
-- Queries
-- ============================================================

-- Return the effective low-range multiplier (1.0 in high, config value in low).
function TransferCase.getLowRangeMult(tc, config)
	return tc and tc.low_range_active and config.low_range_ratio or 1.0
end

-- Return the effective torque split [front, rear] for a given output torque.
function TransferCase.splitTorque(tc, config, total_torque)
	local front_frac = config.torque_split_front or 0.5
	local rear_frac = 1.0 - front_frac

	-- In 2H, all torque goes to rear
	if tc.mode == "2H" then
		return 0, total_torque
	end

	-- With center coupling, distribute per split ratio
	local center = clamp(tc.center_coupling or 0, 0, 1)
	local front = total_torque * front_frac * center
	local rear = total_torque * (1.0 - front_frac) * center + total_torque * (1.0 - center)
	return front, rear
end

-- Return current drivetrain efficiency factor
function TransferCase.getEfficiency(tc, config)
	local e = config.efficiency_2h or 0.92
	if tc.mode == "4H" then
		e = config.efficiency_4h or 0.88
	elseif tc.mode == "4L" then
		e = config.efficiency_4l or 0.85
	end
	return clamp(e, 0.5, 1.0)
end

-- Front drive shaft receives torque from this coupling
function TransferCase.frontDriveAmount(tc)
	if tc.mode == "2H" then return 0 end
	if tc.mode == "Neutral" then return 0 end
	if tc.mode == "4A" then return tc.center_coupling or 0 end
	return 1.0  -- 4H & 4L: always full drive
end

-- Rear differential mode for the given transfer-case and driver request
function TransferCase.rearDiffMode(tc, config)
	if config.rear_diff_type == "E-Locker" and tc.rear_locker_engaged then
		return "Locked"
	elseif config.rear_diff_type == "LimitedSlip" then
		return "LimitedSlip"
	end
	return "Open"
end

-- Front differential mode
function TransferCase.frontDiffMode(tc, config)
	local mode = tc.mode
	-- In 2H with FAD, front is completely disconnected
	if mode == "2H" and config.front_axle_disconnect then
		return "Disconnected"
	end
	if config.front_diff_type == "Open" then
		return "Open"
	elseif config.front_diff_type == "LimitedSlip" then
		return "LimitedSlip"
	end
	return "Open"
end

-- ============================================================
-- Telemetry
-- ============================================================

function TransferCase.telemetry(tc)
	return {
		transfer_mode = tc.mode,
		transfer_target = tc.target_mode,
		center_coupling = tc.center_coupling,
		rear_locker = tc.rear_locker_engaged,
		low_range_active = tc.low_range_active,
		front_engaged = tc.front_engaged,
	}
end

return TransferCase