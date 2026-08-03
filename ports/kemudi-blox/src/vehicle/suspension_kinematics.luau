--[[
	Suspension kinematics — camber, toe, roll center, anti-dive/squat, scrub radius.
	Pure functions, no Roblox API dependencies.
	NaN guards on all public entry points.
]]

local SuspensionKinematics = {}

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
-- Config factory
-- ============================================================

function SuspensionKinematics.newConfig(overrides: { [string]: any }?): { [string]: any }
	assert(type(overrides) == "table", "suspension kinematics parameters must be authored by the vehicle asset")
	return overrides
end

-- ============================================================
-- Camber
-- ============================================================

-- wheel_travel_mm: positive = compression (wheel moves up relative to chassis)
function SuspensionKinematics.getCamber(config, wheel_travel_mm: number): number
	local travel = clamp(safeFinite(wheel_travel_mm), -config.max_extension, config.max_compression)
	local camber = config.camber_at_rest + config.camber_gain * travel
	return safeFinite(clamp(camber, -8, 8))
end

-- ============================================================
-- Toe (bump steer)
-- ============================================================

function SuspensionKinematics.getToe(config, wheel_travel_mm: number): number
	local travel = clamp(safeFinite(wheel_travel_mm), -config.max_extension, config.max_compression)
	local toe = config.toe_at_rest + config.toe_gain * travel
	return safeFinite(clamp(toe, -3, 3))
end

-- ============================================================
-- Roll center
-- ============================================================

function SuspensionKinematics.getRollCenter(config, body_roll_deg: number)
	local roll = safeFinite(body_roll_deg)
	return {
		height = safeFinite(config.roll_center_height),
		lateral_offset = safeFinite(config.roll_center_migration * roll),
	}
end

-- ============================================================
-- Anti-dive / anti-squat
-- ============================================================

-- Returns the fraction of longitudinal force routed through suspension geometry
-- rather than springs. Caller subtracts from spring load on the affected axle.
-- contact_height: vertical distance from ground to chassis CoM (meters)
-- wheelbase: distance between front and rear axles (meters)
function SuspensionKinematics.getAntiDiveForce(config, braking_force: number, contact_height: number, wheelbase: number): number
	local f = safeFinite(braking_force)
	local h = safeFinite(contact_height)
	local wb = safeFinite(wheelbase)
	if wb <= 0 then return 0 end
	-- Geometry factor scales the percentage by the pitch moment arm ratio
	local geometry_ratio = clamp(h / wb, 0, 1)
	return safeFinite(config.anti_dive_pct * geometry_ratio * f)
end

function SuspensionKinematics.getAntiSquatForce(config, acceleration_force: number, contact_height: number, wheelbase: number): number
	local f = safeFinite(acceleration_force)
	local h = safeFinite(contact_height)
	local wb = safeFinite(wheelbase)
	if wb <= 0 then return 0 end
	local geometry_ratio = clamp(h / wb, 0, 1)
	return safeFinite(config.anti_squat_pct * geometry_ratio * f)
end

-- ============================================================
-- Scrub radius
-- ============================================================

-- Simplified scrub radius with KPI effect. Actual scrub varies with camber
-- and caster; this captures the dominant KPI contribution.
function SuspensionKinematics.getScrubRadius(config, wheel_travel_mm: number): number
	local travel = safeFinite(wheel_travel_mm)
	local kpi_rad = math.rad(safeFinite(config.kingpin_inclination))
	local scrub = config.scrub_radius * (1 + 0.01 * math.sin(kpi_rad) * clamp(travel / 100, -1, 1))
	return safeFinite(math.max(scrub, 0))
end

-- ============================================================
-- Apply kinematics to suspension state
-- ============================================================

-- Modifies suspension_state in-place with camber and toe values.
-- suspension_state must have: compression (0-1 fraction) from the spring solver.
-- These values are consumed by the tire model for accurate slip computation.
function SuspensionKinematics.applyKinematics(suspension_state, config, wheel_index: number)
	if not suspension_state then return end
	local compression_mm = safeFinite((suspension_state.compression or 0) * config.max_compression)
	suspension_state.camber = SuspensionKinematics.getCamber(config, compression_mm)
	suspension_state.toe = SuspensionKinematics.getToe(config, compression_mm)
	suspension_state.scrub_radius = SuspensionKinematics.getScrubRadius(config, compression_mm)
end

return SuspensionKinematics
