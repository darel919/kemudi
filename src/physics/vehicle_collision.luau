--[[
	Vehicle-to-vehicle collision module.
	Handles node-vs-node contact between different vehicles using a
	spatial hash broadphase. Installed as World methods; see init.luau
	for wiring.

	ALL CONFIG MUST BE PROVIDED BY VEHICLE ASSET / NETWORK.
	No vehicle-specific defaults in engine code.
--]]

local Module = {}

local function nan_guard(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

function Module.install(World, Math)

-- ── State initialization ────────────────────────────────────────────────
-- Spatial hash grid for broadphase
-- ALL values MUST come from vehicle asset / map data
function World:setupVehicleCollision()
	-- Spatial hash cell size (meters) - from vehicle asset
	self.cellSize = self.cellSize
	-- Grid half-extent (meters) - from map data
	self.gridExtent = self.gridExtent
	-- Collision restitution (0-1) - from vehicle asset
	self.collisionRestitution = self.collisionRestitution
	-- Collision friction (0-1) - from vehicle asset
	self.collisionFriction = self.collisionFriction
	-- Damage threshold (impulse in N) - from vehicle asset
	self.collisionDamageThreshold = self.collisionDamageThreshold
end

-- Identity - set by network layer
World.vehicleId = nil
World.otherVehicles = {} -- [id] = { nodes = {{x,y,z}...}, mass = N, lastUpdate = t }
World.impactImpulses = {} -- per-node inter-vehicle impulse vectors

function World:_ensureCollisionState()
	if self.cellSize == nil then
		self:setupVehicleCollision()
	end
end

-- ── Public API ──────────────────────────────────────────────────────────

function World:setVehicleId(id)
	self.vehicleId = id
end

function World:registerOtherVehicle(id, nodePositions, totalMass)
	if id == nil then return end
	if type(nodePositions) ~= "table" or #nodePositions == 0 then return end
	if not (totalMass > 0 and totalMass == totalMass) then totalMass = 1 end
	self.otherVehicles[id] = {
		nodes = nodePositions,
		mass = totalMass,
		lastUpdate = self.time or 0,
	}
end

function World:unregisterOtherVehicle(id)
	self.otherVehicles[id] = nil
end

-- ── Broadphase ──────────────────────────────────────────────────────────

local function gridKey(gx, gz)
	return gx * 100003 + gz -- cheap hash; 100003 is prime
end

function World:_buildSpatialHash()
	self:_ensureCollisionState()
	if not self.cellSize or not self.gridExtent then return {} end

	local grid = {}
	local cellSize = self.cellSize
	local extent = self.gridExtent
	local invCell = 1 / math.max(cellSize, 1e-4)

	-- Helper: insert one node entry into the grid
	local function insertEntry(vId, nodeIdx, x, y, z, mass)
		x = nan_guard(x, 0)
		y = nan_guard(y, 0)
		z = nan_guard(z, 0)
		if not (x == x) then return end -- skip NaN
		local gx = math.floor((x + extent) * invCell)
		local gz = math.floor((z + extent) * invCell)
		-- Clamp to valid range
		local maxCell = math.floor(extent * invCell * 2)
		if gx < 0 then gx = 0 elseif gx > maxCell then gx = maxCell end
		if gz < 0 then gz = 0 elseif gz > maxCell then gz = maxCell end
		local key = gridKey(gx, gz)
		local cell = grid[key]
		if not cell then
			cell = {}
			grid[key] = cell
		end
		cell[#cell + 1] = {
			vehicleId = vId,
			nodeIndex = nodeIdx,
			x = x, y = y, z = z,
			mass = math.max(mass, 1e-4),
		}
	end

	-- Insert self nodes
	local nodes = self.nodes
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			insertEntry(self.vehicleId, i, n.x, n.y, n.z, n.mass)
		end
	end

	-- Insert other vehicles
	for vId, vData in self.otherVehicles do
		local vNodes = vData.nodes
		local nodeMass = if #vNodes > 0 then vData.mass / #vNodes else 0
		for j = 1, #vNodes do
			local p = vNodes[j]
			if p and p[1] == p[1] and p[3] == p[3] then
				insertEntry(vId, j, p[1], p[2] or 0, p[3], nodeMass)
			end
		end
	end

	return grid
end

-- ── Narrowphase & Contact ──────────────────────────────────────────────

-- Contact radius for collision detection (from vehicle asset)
function World:_collisionContactRadius()
	return self.collision_contact_radius or 0.2
end

function World:_solveVehicleCollisions(dt)
	self:_ensureCollisionState()
	if not (dt > 0 and dt == dt) then return end

	-- Required config check
	if not self.cellSize or not self.gridExtent or not self.collisionRestitution or not self.collisionFriction then
		return
	end

	local grid = self:_buildSpatialHash()
	local nodes = self.nodes
	local contactRadius = self:_collisionContactRadius()
	local restitution = self.collisionRestitution
	local friction = self.collisionFriction
	local damageThreshold = self.collisionDamageThreshold or 0

	-- Clear previous impulses
	self.impactImpulses = {}

	-- For each cell, check self vs other nodes
	for key, cell in pairs(grid) do
		if #cell > 1 then
			for i = 1, #cell do
				local a = cell[i]
				if a.vehicleId == self.vehicleId then
					for j = 1, #cell do
						local b = cell[j]
						if b.vehicleId ~= self.vehicleId then
							-- Potential collision
							local dx = b.x - a.x
							local dy = b.y - a.y
							local dz = b.z - a.z
							local distSq = dx*dx + dy*dy + dz*dz
							local minDist = contactRadius * 2

							if distSq < minDist * minDist and distSq > 1e-8 then
								local dist = math.sqrt(distSq)
								local nx, ny, nz = dx/dist, dy/dist, dz/dist

								-- Relative velocity
								local na = nodes[a.nodeIndex]
								local nb_vx, nb_vy, nb_vz = 0, 0, 0
								-- Other vehicle velocity would come from network state
								-- For now assume stationary (or pass in via otherVehicles data)

								if na then
									local rvx = na.vx - nb_vx
									local rvy = na.vy - nb_vy
									local rvz = na.vz - nb_vz
									local vRel = rvx*nx + rvy*ny + rvz*nz

									if vRel < 0 then -- approaching
										-- Impulse: j = -(1+e) * vRel / (1/m1 + 1/m2)
										local m1 = na.mass
										local m2 = b.mass
										local invMassSum = 1/m1 + 1/m2
										if invMassSum > 0 then
											local j = -(1 + restitution) * vRel / invMassSum
											if damageThreshold > 0 and j > damageThreshold then
												-- Damage event - flag for visual/deformation system
												self.impactImpulses[a.nodeIndex] = {
													impulse = j,
													direction = {nx, ny, nz},
													otherVehicle = b.vehicleId,
													damage = math.min(j / damageThreshold, 10)
												}
											end

											-- Apply impulse
											local ix = j * nx
											local iy = j * ny
											local iz = j * nz

											na.vx = na.vx + ix / m1
											na.vy = na.vy + iy / m1
											na.vz = na.vz + iz / m1

											-- Friction impulse (tangential)
											local tvx = rvx - vRel * nx
											local tvy = rvy - vRel * ny
											local tvz = rvz - vRel * nz
											local tSpeed = math.sqrt(tvx*tvx + tvy*tvy + tvz*tvz)
											if tSpeed > 1e-6 then
												local jt = math.min(friction * math.abs(j), tSpeed * m1 * m2 / (m1 + m2) / dt)
												na.vx = na.vx - jt * tvx / tSpeed / m1
												na.vy = na.vy - jt * tvy / tSpeed / m1
												na.vz = na.vz - jt * tvz / tSpeed / m1
											end
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end
end

function World:_collideWithOtherVehicles(dt)
	self:_solveVehicleCollisions(dt)
end

-- Telemetry
function World:getCollisionTelemetry()
	if not self.impactImpulses then return {} end
	local totalImpulse = 0
	local maxImpulse = 0
	local count = 0
	for _, imp in pairs(self.impactImpulses) do
		totalImpulse = totalImpulse + (imp.impulse or 0)
		maxImpulse = math.max(maxImpulse, imp.impulse or 0)
		count = count + 1
	end
	return {
		impact_count = count,
		total_impulse = totalImpulse,
		max_impulse = maxImpulse,
	}
end

end -- Module.install

return Module