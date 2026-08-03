--[[
	Aerodynamic model for soft-body vehicle simulation.
	Replaces the old body-level drag with per-node aerodynamic forces,
	ground-effect lift/downforce, pitch moments, and sideslip drag.

	ALL AERO COEFFICIENTS MUST BE PROVIDED BY VEHICLE ASSET.
	No vehicle-specific defaults in engine code.

	Called at step 6 (after velocity integration), so it modifies
	velocities directly as additive corrections (Δv = F/m × dt).
	The XPBD constraint solver and beam mesh then resist deformation
	under asymmetric aero loads, producing realistic body flex at speed.
--]]

local Module = {}

-- Aerodynamic constants (universal physics, NOT vehicle-specific)
local MIN_SPEED = 0.5              -- ignore aero below this speed (m/s)
local WHEEL_NODE_END = 4           -- nodes 1-4 are suspension/wheel mounts
local REAR_HEIGHT_REF = 0.3        -- reference ride height for ground effect (m)
local GROUND_EFFECT_MIN = 0.15
local GROUND_EFFECT_MAX = 2.0

local function nan_guard(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end -- NaN
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

local function sqrt_safe(x)
	if x <= 0 then return 0 end
	return math.sqrt(x)
end

function Module.install(World)

-- Full aerodynamic model replacing the old body-level drag.
function World:_applyDrag()
	local nodes = self.nodes
	local count = #nodes
	if count == 0 then return end

	local dt = self.fixedDt
	if not (dt > 0 and dt == dt) then return end

	-- Air density (from vehicle asset or environment)
	local rho = nan_guard(self.airDensity, 1.225)
	-- Frontal area (from vehicle asset)
	local frontalArea = nan_guard(self.frontalArea, 0)

	-- Vehicle-specific aero coefficients — MUST come from vehicle asset
	-- If missing, that aero effect is disabled (no effect)
	local aeroCd = self.aeroCd
	local aeroCl = self.aeroCl
	local aeroCs = self.aeroCs
	local aeroCm = self.aeroCm
	local aeroWingArea = self.aeroWingArea
	local aeroWingCl = self.aeroWingCl
	local aeroGroundRefHeight = self.aeroGroundRefHeight
	local aeroPitchBrakeCoeff = self.aeroPitchBrakeCoeff
	local aeroPitchPowerCoeff = self.aeroPitchPowerCoeff
	local aeroNodeDrag = self.aeroNodeDrag

	-- Current vehicle state
	local speed = nan_guard(self.vehicle_speed, 0)
	if speed < MIN_SPEED then return end

	-- Dynamic pressure: q = 0.5 * rho * v^2
	local q = 0.5 * rho * speed * speed

	-- Pitch moment from brake/throttle (configurable per vehicle)
	local brakePitch = 0
	local throttlePitch = 0
	if self.controls then
		brakePitch = nan_guard(aeroPitchBrakeCoeff, 0) * (self.controls.brake or 0)
		throttlePitch = nan_guard(aeroPitchPowerCoeff, 0) * (self.controls.throttle or 0)
	end
	local pitchMoment = (brakePitch - throttlePitch) * q * frontalArea

	-- Ground effect: more downforce when closer to ground
	-- Compute average height of body nodes (excluding wheels)
	local bodyHeightSum = 0
	local bodyCount = 0
	for i = WHEEL_NODE_END + 1, count do
		local n = nodes[i]
		if not n.fixed and n.y then
			bodyHeightSum = bodyHeightSum + n.y
			bodyCount = bodyCount + 1
		end
	end
	local avgHeight = (bodyCount > 0) and (bodyHeightSum / bodyCount) or REAR_HEIGHT_REF
	local groundRef = nan_guard(self.aeroGroundRefHeight, REAR_HEIGHT_REF)
	local groundEffectMin = nan_guard(self.aeroGroundEffectMin, GROUND_EFFECT_MIN)
	local groundEffectMax = nan_guard(self.aeroGroundEffectMax, GROUND_EFFECT_MAX)
	local groundEffect = 1.0
	if avgHeight < groundRef * groundEffectMax then
		groundEffect = clamp(groundRef / math.max(avgHeight, groundRef * groundEffectMin), 1.0, groundEffectMax)
	end

	local v_mag = math.max(speed, MIN_SPEED)

	-- Per-node aerodynamic forces
	for i = 1, count do
		local n = nodes[i]
		if n.fixed then continue end

		-- Velocity components in local vehicle frame
		local vx, vy, vz = n.vx, n.vy, n.vz
		local localSpeed = sqrt_safe(vx*vx + vy*vy + vz*vz)
		local effectiveSpeed = math.max(localSpeed, v_mag)

		-- Drag: F_d = -0.5 * rho * v^2 * Cd * A * sign(v)
		-- Frontal drag on wheel nodes (exposed)
		if i <= WHEEL_NODE_END then
			local nodeCd = nan_guard(aeroNodeDrag, nan_guard(aeroCd, 0))
			if nodeCd > 0 and localSpeed > 0 then
				local drag = 0.5 * rho * effectiveSpeed * effectiveSpeed * nodeCd * (frontalArea * 0.25)
				n.vx = n.vx - drag * (vx / localSpeed) * dt / math.max(n.mass, 0.01)
				n.vy = n.vy - drag * (vy / localSpeed) * dt / math.max(n.mass, 0.01)
				n.vz = n.vz - drag * (vz / localSpeed) * dt / math.max(n.mass, 0.01)
			end
		else
			-- Body nodes: Cd based on orientation (simplified)
			local nodeCd = nan_guard(aeroNodeDrag, nan_guard(aeroCd, 0))
			if nodeCd > 0 and localSpeed > 0 then
				local drag = 0.5 * rho * effectiveSpeed * effectiveSpeed * nodeCd * (frontalArea / math.max(count - 4, 1))
				n.vx = n.vx - drag * (vx / localSpeed) * dt / math.max(n.mass, 0.01)
				n.vy = n.vy - drag * (vy / localSpeed) * dt / math.max(n.mass, 0.01)
				n.vz = n.vz - drag * (vz / localSpeed) * dt / math.max(n.mass, 0.01)
			end
		end

		-- Downforce/lift: F_lift = 0.5 * rho * v^2 * Cl * A * groundEffect
		local cl = nan_guard(aeroCl, 0)
		if cl ~= 0 and effectiveSpeed > 0 then
			local lift = q * cl * (frontalArea / math.max(count, 1)) * groundEffect
			-- Apply as vertical force (negative Cl = downforce = positive Y force)
			n.vy = n.vy + (-lift) * dt / math.max(n.mass, 0.01)
		end

		-- Side force from sideslip/yaw
		local cs = nan_guard(aeroCs, 0)
		if cs ~= 0 and effectiveSpeed > 0 then
			-- Approximate sideslip from velocity vs forward direction
			-- This is simplified; real implementation would use vehicle orientation
			local sideForce = q * cs * (frontalArea / math.max(count, 1))
			-- Apply perpendicular to velocity (simplified as X/Z component)
			if math.abs(vx) > 0.1 or math.abs(vz) > 0.1 then
				local sideDir = math.atan2(vz, vx)
				n.vx = n.vx + sideForce * math.cos(sideDir + math.pi/2) * dt / math.max(n.mass, 0.01)
				n.vz = n.vz + sideForce * math.sin(sideDir + math.pi/2) * dt / math.max(n.mass, 0.01)
			end
		end
	end

	-- Wing downforce (separate from body Cl)
	local wingCl = nan_guard(aeroWingCl, 0)
	local wingArea = nan_guard(aeroWingArea, 0)
	if wingCl ~= 0 and wingArea > 0 and v_mag > 0 and count > WHEEL_NODE_END then
		local wingLift = q * wingCl * wingArea * groundEffect
		-- Apply to the rear-most body nodes; wheel nodes are 1-4.
		for i = math.max(WHEEL_NODE_END + 1, count - 1), count do
			local n = nodes[i]
			if n and not n.fixed then
				n.vy = n.vy + (-wingLift * 0.5) * dt / math.max(n.mass, 0.01)
			end
		end
	end

	-- Pitch moment: apply as differential vertical forces on front/rear
	if pitchMoment ~= 0 then
		-- Distribute pitch moment as vertical force couple
		local frontNodes = {1, 2}
		local rearNodes = {3, 4}
		local forcePerNode = pitchMoment / 4 -- simplified distribution
		for _, idx in ipairs(frontNodes) do
			if idx <= count then
				local n = nodes[idx]
				if n and not n.fixed then
					n.vy = n.vy + forcePerNode * dt / math.max(n.mass, 0.01)
				end
			end
		end
		for _, idx in ipairs(rearNodes) do
			if idx <= count then
				local n = nodes[idx]
				if n and not n.fixed then
					n.vy = n.vy - forcePerNode * dt / math.max(n.mass, 0.01)
				end
			end
		end
	end
end

end -- Module.install

return Module