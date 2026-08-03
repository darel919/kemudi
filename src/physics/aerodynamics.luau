--[[
	Aerodynamic model for soft-body vehicle simulation.
	Replaces the old body-level drag with per-node aerodynamic forces,
	triangle-based surface aerodynamics, ground-effect lift/downforce,
	pitch moments, and sideslip drag.

	When triangle aero groups are configured (via addTriangleAero),
	drag and lift are computed per-surface using surface normals and areas.
	Falls back to uniform per-node distribution when no groups are set.

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

local function cross3(ax, ay, az, bx, by, bz)
	return ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx
end

local function dot3(ax, ay, az, bx, by, bz)
	return ax*bx + ay*by + az*bz
end

function Module.install(World, Math)

-- Triangle aero surface groups: { [triIndex] = { group, dragCoef, liftCoef } }
-- Groups: "body", "wing", "underbody"
function World:_ensureTriangleAero()
	if self._triangleAeroGroups then return end
	self._triangleAeroGroups = {}
end

-- Register a triangle as an aero surface with drag/lift coefficients.
-- triIndex: 1-based index into self.triangles
-- group: "body" | "wing" | "underbody"
-- dragCoef: per-surface drag coefficient (typically 0.2-0.5)
-- liftCoef: per-surface lift coefficient (negative = downforce)
function World:addTriangleAero(triIndex, group, dragCoef, liftCoef)
	self:_ensureTriangleAero()
	self._triangleAeroGroups[triIndex] = {
		group = group or "body",
		dragCoef = nan_guard(dragCoef, 0.3),
		liftCoef = nan_guard(liftCoef, 0),
	}
end

-- Full aerodynamic model replacing the old body-level drag.
function World:_applyDrag()
	local nodes = self.nodes
	local count = #nodes
	if count == 0 then return end

	local dt = self.fixedDt
	if not (dt > 0 and dt == dt) then return end

	-- Air density (from vehicle asset or environment)
	local rho = self.airDensity or 1.225
	-- Frontal area (from vehicle asset)
	local frontalArea = self.frontalArea or 2.2

	-- Vehicle-specific aero coefficients — MUST come from vehicle asset
	local aeroCd = self.aeroCd
	local aeroCl = self.aeroCl
	local aeroCs = self.aeroCs
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
	local bodyHeightSum = 0
	local bodyCount = 0
	for i = WHEEL_NODE_END + 1, count do
		local n = nodes[i]
		if not n.fixed and n.y then
			bodyHeightSum = bodyHeightSum + n.y
			bodyCount = bodyCount + 1
		end
	end
	local groundRef = self.aeroGroundRefHeight or 0.5
	local avgHeight = (bodyCount > 0) and (bodyHeightSum / bodyCount) or groundRef
	local groundEffectMin = self.aeroGroundEffectMin or 0
	local groundEffectMax = self.aeroGroundEffectMax or 0
	local groundEffect = 1.0
	if avgHeight < groundRef * groundEffectMax then
		groundEffect = clamp(groundRef / math.max(avgHeight, groundRef * groundEffectMin), 1.0, groundEffectMax)
	end

	local v_mag = math.max(speed, MIN_SPEED)

	-- ============================================================
	-- Triangle-based aerodynamic forces (when configured)
	-- ============================================================
	self:_ensureTriangleAero()
	local triAero = self._triangleAeroGroups
	local triangles = self.triangles
	local hasTriangleAero = next(triAero) ~= nil

	if hasTriangleAero and triangles and #triangles > 0 then
		-- Per-node accumulated triangle aero forces (applied after loop)
		local triFx = table.create(count, 0)
		local triFy = table.create(count, 0)
		local triFz = table.create(count, 0)

		for ti, tri in ipairs(triangles) do
			local aeroGroup = triAero[ti]
			if not aeroGroup then continue end

			local ai, bi, ci = tri.a + 1, tri.b + 1, tri.c + 1
			if ai > count or bi > count or ci > count then continue end

			local na, nb, nc = nodes[ai], nodes[bi], nodes[ci]

			-- Compute surface normal and area from triangle edges
			local abx, aby, abz = nb.x - na.x, nb.y - na.y, nb.z - na.z
			local acx, acy, acz = nc.x - na.x, nc.y - na.y, nc.z - na.z
			local nx, ny, nz = cross3(abx, aby, abz, acx, acy, acz)
			local area2 = sqrt_safe(nx*nx + ny*ny + nz*nz)
			if area2 < 1e-10 then continue end
			nx, ny, nz = nx/area2, ny/area2, nz/area2
			local surfaceArea = area2 * 0.5

			-- Compute average velocity of the three nodes
			local avgVx = (na.vx + nb.vx + nc.vx) / 3
			local avgVy = (na.vy + nb.vy + nc.vy) / 3
			local avgVz = (na.vz + nb.vz + nc.vz) / 3
			local avgSpeed = sqrt_safe(avgVx*avgVx + avgVy*avgVy + avgVz*avgVz)
			if avgSpeed < MIN_SPEED then continue end

			-- Velocity component along surface normal (approach speed)
			local approachSpeed = math.abs(dot3(avgVx, avgVy, avgVz, nx, ny, nz))
			local effSpeed = math.max(avgSpeed, approachSpeed)

			-- Dynamic pressure × surface area
			local qA = q * surfaceArea

			-- Drag: opposing velocity direction, scaled by Cd
			local dragCd = aeroGroup.dragCoef
			if dragCd > 0 then
				local dragMag = qA * dragCd * groundEffect
				-- Drag opposes velocity direction
				local dragX = -dragMag * (avgVx / avgSpeed)
				local dragY = -dragMag * (avgVy / avgSpeed)
				local dragZ = -dragMag * (avgVz / avgSpeed)
				local third = 1/3
				triFx[ai] += dragX * third; triFy[ai] += dragY * third; triFz[ai] += dragZ * third
				triFx[bi] += dragX * third; triFy[bi] += dragY * third; triFz[bi] += dragZ * third
				triFx[ci] += dragX * third; triFy[ci] += dragY * third; triFz[ci] += dragZ * third
			end

			-- Lift: along surface normal, scaled by Cl
			local liftCl = aeroGroup.liftCoef
			if liftCl ~= 0 then
				local liftMag = qA * liftCl * groundEffect
				local liftX = liftMag * nx
				local liftY = liftMag * ny
				local liftZ = liftMag * nz
				local third = 1/3
				triFx[ai] += liftX * third; triFy[ai] += liftY * third; triFz[ai] += liftZ * third
				triFx[bi] += liftX * third; triFy[bi] += liftY * third; triFz[bi] += liftZ * third
				triFx[ci] += liftX * third; triFy[ci] += liftY * third; triFz[ci] += liftZ * third
			end

			-- Ground effect multiplier for underbody group
			if aeroGroup.group == "underbody" and groundEffect > 1.0 then
				-- Extra downforce from ground proximity (Venturi effect)
				local venturiMag = qA * (groundEffect - 1.0) * 0.5
				local ventX = venturiMag * nx
				local ventY = venturiMag * ny
				local ventZ = venturiMag * nz
				local third = 1/3
				triFx[ai] += ventX * third; triFy[ai] += ventY * third; triFz[ai] += ventZ * third
				triFx[bi] += ventX * third; triFy[bi] += ventY * third; triFz[bi] += ventZ * third
				triFx[ci] += ventX * third; triFy[ci] += ventY * third; triFz[ci] += ventZ * third
			end
		end

		-- Apply triangle aero forces as velocity corrections
		for i = 1, count do
			local n = nodes[i]
			if not n.fixed and n.mass > 0 then
				local invMass = 1 / math.max(n.mass, 0.01)
				n.vx += triFx[i] * dt * invMass
				n.vy += triFy[i] * dt * invMass
				n.vz += triFz[i] * dt * invMass
			end
		end
	else
		-- ============================================================
		-- Fallback: uniform per-node aerodynamic forces (no triangle groups)
		-- ============================================================
		for i = 1, count do
			local n = nodes[i]
			if n.fixed then continue end

			local vx, vy, vz = n.vx, n.vy, n.vz
			local localSpeed = sqrt_safe(vx*vx + vy*vy + vz*vz)
			local effectiveSpeed = math.max(localSpeed, v_mag)

			-- Drag on wheel nodes (exposed)
			if i <= WHEEL_NODE_END then
				local nodeCd = nan_guard(aeroNodeDrag, nan_guard(aeroCd, 0))
				if nodeCd > 0 and localSpeed > 0 then
					local drag = 0.5 * rho * effectiveSpeed * effectiveSpeed * nodeCd * (frontalArea * 0.25)
					n.vx = n.vx - drag * (vx / localSpeed) * dt / math.max(n.mass, 0.01)
					n.vy = n.vy - drag * (vy / localSpeed) * dt / math.max(n.mass, 0.01)
					n.vz = n.vz - drag * (vz / localSpeed) * dt / math.max(n.mass, 0.01)
				end
			else
				-- Body nodes
				local nodeCd = nan_guard(aeroNodeDrag, nan_guard(aeroCd, 0))
				if nodeCd > 0 and localSpeed > 0 then
					local drag = 0.5 * rho * effectiveSpeed * effectiveSpeed * nodeCd * (frontalArea / math.max(count - 4, 1))
					n.vx = n.vx - drag * (vx / localSpeed) * dt / math.max(n.mass, 0.01)
					n.vy = n.vy - drag * (vy / localSpeed) * dt / math.max(n.mass, 0.01)
					n.vz = n.vz - drag * (vz / localSpeed) * dt / math.max(n.mass, 0.01)
				end
			end

			-- Downforce/lift
			local cl = nan_guard(aeroCl, 0)
			if cl ~= 0 and effectiveSpeed > 0 then
				local lift = q * cl * (frontalArea / math.max(count, 1)) * groundEffect
				n.vy = n.vy + lift * dt / math.max(n.mass, 0.01)
			end

			-- Side force from sideslip/yaw
			local cs = nan_guard(aeroCs, 0)
			if cs ~= 0 and effectiveSpeed > 0 then
				local sideForce = q * cs * (frontalArea / math.max(count, 1))
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
			for i = math.max(WHEEL_NODE_END + 1, count - 1), count do
				local n = nodes[i]
				if n and not n.fixed then
					n.vy = n.vy + (wingLift * 0.5) * dt / math.max(n.mass, 0.01)
				end
			end
		end
	end

	-- Pitch moment: apply as differential vertical forces on front/rear
	if pitchMoment ~= 0 then
		local frontNodes = {1, 2}
		local rearNodes = {3, 4}
		local forcePerNode = pitchMoment / 4
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
