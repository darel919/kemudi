--[[
	Subframe compliance via rubber bushings.
	Bounded spring-damper connections between chassis nodes that
	only activate outside a configurable deadzone (play zone).
	Bushings degrade under sustained high stress
	and can be destroyed for damage visualization.

	ALL CONFIG MUST BE PROVIDED BY VEHICLE ASSET.
	No vehicle-specific defaults in engine code.
--]]

local Module = {}

local EPS = 1e-8

local function safeLength3(x, y, z)
	local len = math.sqrt(x * x + y * y + z * z)
	if not (len > EPS) or len ~= len then return 0 end
	return len
end

local function nan_guard(v, fallback)
	if v ~= v then return fallback end
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

function Module.install(World, Math)

	-- Lazy init: ensure bushing state exists on any world instance.
	local function ensureBushings(self)
		if not self.bushings then
			self.bushings = {}
			self.bushingDamage = 0.0
		end
	end

	-- Add a compliant bushing between two nodes (0-based indices).
	-- config: { stiffness?, damping?, deadzone? }
	-- All values MUST come from vehicle asset. If missing, bushing has no effect.
	function World:addBushing(nodeA_idx, nodeB_idx, config)
		ensureBushings(self)
		local nodes = self.nodes
		local ai, bi = nodeA_idx + 1, nodeB_idx + 1
		if ai > #nodes or bi > #nodes or ai < 1 or bi < 1 then return end
		if ai == bi then return end

		local na, nb = nodes[ai], nodes[bi]
		if not na or not nb then return end

		local cfg = config or {}
		-- No defaults - if missing, no effect
		local stiffness = cfg.stiffness
		local damping = cfg.damping
		local deadzone = cfg.deadzone

		if stiffness == nil or not (stiffness > 0 and stiffness == stiffness) then return end
		if damping == nil or not (damping >= 0 and damping == damping) then damping = 0 end
		if deadzone == nil or not (deadzone >= 0 and deadzone == deadzone) then deadzone = 0 end

		local dx, dy, dz = nb.x - na.x, nb.y - na.y, nb.z - na.z
		local restLen = safeLength3(dx, dy, dz)
		if restLen <= 0 then return end

		self.bushings[#self.bushings + 1] = {
			nodeA = nodeA_idx,
			nodeB = nodeB_idx,
			restLength = restLen,
			stiffness_x = stiffness,
			stiffness_y = stiffness,
			stiffness_z = stiffness,
			damping_x = damping,
			damping_y = damping,
			damping_z = damping,
			deadzone = deadzone,
			stress = 0,
			damage = 0,
		}
	end

	-- Add bushing with explicit per-axis stiffness/damping.
	function World:addBushingXYZ(nodeA_idx, nodeB_idx, sx, sy, sz, dx, dy, dz, deadzone)
		ensureBushings(self)
		local nodes = self.nodes
		local ai, bi = nodeA_idx + 1, nodeB_idx + 1
		if ai > #nodes or bi > #nodes or ai < 1 or bi < 1 then return end
		if ai == bi then return end

		local na, nb = nodes[ai], nodes[bi]
		if not na or not nb then return end

		-- All values required from vehicle asset
		if sx == nil or sy == nil or sz == nil or dx == nil or dy == nil or dz == nil then
			return
		end
		if deadzone == nil then deadzone = 0 end

		local dxx, dyy, dzz = nb.x - na.x, nb.y - na.y, nb.z - na.z
		local restLen = safeLength3(dxx, dyy, dzz)
		if restLen <= 0 then return end

		self.bushings[#self.bushings + 1] = {
			nodeA = nodeA_idx,
			nodeB = nodeB_idx,
			restLength = restLen,
			stiffness_x = sx,
			stiffness_y = sy,
			stiffness_z = sz,
			damping_x = dx,
			damping_y = dy,
			damping_z = dz,
			deadzone = deadzone,
			stress = 0,
			damage = 0,
		}
	end

	-- Solve all bushings - applies forces to nodes
	function World:_solveBushings(dt)
		if not self.bushings or #self.bushings == 0 then return end
		if not (dt > 0 and dt == dt) then return end

		local nodes = self.nodes
		local invDt = 1 / dt

		for _, b in ipairs(self.bushings) do
			local na = nodes[b.nodeA + 1]
			local nb = nodes[b.nodeB + 1]
			if not na or not nb or na.fixed or nb.fixed then
				-- If one node is fixed, the other can still be moved relative to fixed point
				-- For now skip if either fixed (can be extended)
				continue
			end

			local dx = nb.x - na.x
			local dy = nb.y - na.y
			local dz = nb.z - na.z
			local len = safeLength3(dx, dy, dz)

			if len <= EPS then
				b.stress = 0
			else
				-- Normalized direction
				local nx, ny, nz = dx / len, dy / len, dz / len

				-- Deadzone: no force within play zone
				local delta = len - b.restLength
				if math.abs(delta) < b.deadzone then
					b.stress = 0
				else
					-- Spring force
					local fx = 0
					if delta > b.deadzone then
						fx = (delta - b.deadzone) * b.stiffness_x
					elseif delta < -b.deadzone then
						fx = (delta + b.deadzone) * b.stiffness_x
					end

					-- Damping force
					local vx = nb.vx - na.vx
					local vy = nb.vy - na.vy
					local vz = nb.vz - na.vz
					local vRel = vx * nx + vy * ny + vz * nz
					local fv = -vRel * b.damping_x

					local force = fx + fv
					if force ~= force then force = 0 end -- NaN guard

					-- Apply equal and opposite forces
					na.fx, na.fy, na.fz = na.fx or 0, na.fy or 0, na.fz or 0
					nb.fx, nb.fy, nb.fz = nb.fx or 0, nb.fy or 0, nb.fz or 0
					na.fx = na.fx - force * nx
					na.fy = na.fy - force * ny
					na.fz = na.fz - force * nz
					nb.fx = nb.fx + force * nx
					nb.fy = nb.fy + force * ny
					nb.fz = nb.fz + force * nz

					-- Stress for damage tracking
					b.stress = math.abs(force) / math.max(b.stiffness_x * b.deadzone, 1)

					-- Damage accumulation
					if b.stress > 1.5 then
						b.damage = math.min(b.damage + b.stress * 0.01 * dt, 1)
						if b.damage >= 1.0 then
							-- Bushing destroyed - remove or mark
							b.stiffness_x = 0
							b.stiffness_y = 0
							b.stiffness_z = 0
							b.damping_x = 0
							b.damping_y = 0
							b.damping_z = 0
							self.bushingDamage = math.min(self.bushingDamage + 0.1, 1)
						end
					end
				end
			end
		end
	end

	function World:setupDefaultBushings(definitions)
		for _, definition in ipairs(definitions or {}) do
			self:addBushing(definition.nodeA, definition.nodeB, definition)
		end
	end

	function World:_updateBushings(dt)
		self:_solveBushings(dt)
	end

	function World:_updateBushingDamage(dt)
		if not self.bushings then return end
		for _, b in ipairs(self.bushings) do
			if b.stress <= 1.5 then
				b.damage = math.max(b.damage - dt * 0.1, 0)
			end
		end
	end

	function World:destroyBushing(index)
		local b = self.bushings and self.bushings[index]
		if not b then return end
		b.restLength = 0
		b.stress = 0
		b.damage = 0
		b.stiffness_x, b.stiffness_y, b.stiffness_z = 0, 0, 0
		b.damping_x, b.damping_y, b.damping_z = 0, 0, 0
	end

	-- Telemetry
	function World:getBushingTelemetry()
		if not self.bushings then return {} end
		local totalStress = 0
		local maxStress = 0
		local totalDamage = 0
		for _, b in ipairs(self.bushings) do
			totalStress = totalStress + b.stress
			maxStress = math.max(maxStress, b.stress)
			totalDamage = totalDamage + b.damage
		end
		return {
			count = #self.bushings,
			avg_stress = totalStress / math.max(#self.bushings, 1),
			max_stress = maxStress,
			total_damage = totalDamage,
			overall_damage = self.bushingDamage or 0,
		}
	end

end -- Module.install

return Module