local Module = {}

function Module.install(World, Contract, Math)

-- ============================================================
-- Spatial Hash Broadphase for collision
-- ============================================================

local function hashKey(ix, iy, iz)
	return ix * 73856093 + iy * 19349663 + iz * 83492791
end

local function getCellCoords(x, y, z, cellSize)
	local inv = 1 / cellSize
	return math.floor(x * inv), math.floor(y * inv), math.floor(z * inv)
end

local function buildSpatialHash(primitives, cellSize)
	local hash = {}
	for i, prim in primitives do
		-- prim can be a raw primitive (from collisionBoxes) or a wrapper {type="box", prim=box}
		local raw = prim.prim or prim
		-- Get primitive bounds
		local minX, minY, minZ, maxX, maxY, maxZ
		if raw.radius then
			-- sphere
			minX, minY, minZ = raw.center[1] - raw.radius, raw.center[2] - raw.radius, raw.center[3] - raw.radius
			maxX, maxY, maxZ = raw.center[1] + raw.radius, raw.center[2] + raw.radius, raw.center[3] + raw.radius
		else
			-- box
			minX = raw.center[1] - raw.halfExtents[1]
			minY = raw.center[2] - raw.halfExtents[2]
			minZ = raw.center[3] - raw.halfExtents[3]
			maxX = raw.center[1] + raw.halfExtents[1]
			maxY = raw.center[2] + raw.halfExtents[2]
			maxZ = raw.center[3] + raw.halfExtents[3]
		end
		local cx1, cy1, cz1 = getCellCoords(minX, minY, minZ, cellSize)
		local cx2, cy2, cz2 = getCellCoords(maxX, maxY, maxZ, cellSize)
		for ix = cx1, cx2 do
			for iy = cy1, cy2 do
				for iz = cz1, cz2 do
					local key = hashKey(ix, iy, iz)
					if not hash[key] then hash[key] = {} end
					table.insert(hash[key], raw)
				end
			end
		end
	end
	return hash
end

local function querySpatialHash(hash, x, y, z, radius, cellSize)
	local minX, minY, minZ = x - radius, y - radius, z - radius
	local maxX, maxY, maxZ = x + radius, y + radius, z + radius
	local cx1, cy1, cz1 = getCellCoords(minX, minY, minZ, cellSize)
	local cx2, cy2, cz2 = getCellCoords(maxX, maxY, maxZ, cellSize)
	local results = {}
	for ix = cx1, cx2 do
		for iy = cy1, cy2 do
			for iz = cz1, cz2 do
				local key = hashKey(ix, iy, iz)
				if hash[key] then
					for _, prim in hash[key] do
						table.insert(results, prim)
					end
				end
			end
		end
	end
	return results
end

function World:_rebuildCollisionHash()
	local cellSize = self.cellSize or 4
	-- Combine all static primitives
	local allPrimitives = {}
	for _, box in self.collisionBoxes do table.insert(allPrimitives, { type = "box", prim = box }) end
	for _, sphere in self.collisionSpheres do table.insert(allPrimitives, { type = "sphere", prim = sphere }) end
	if self.mapCollisionBoxes then
		for _, box in self.mapCollisionBoxes do table.insert(allPrimitives, { type = "box", prim = box }) end
	end
	if self.mapCollisionSpheres then
		for _, sphere in self.mapCollisionSpheres do table.insert(allPrimitives, { type = "sphere", prim = sphere }) end
	end
	self.collisionHash = buildSpatialHash(allPrimitives, cellSize)
	self.collisionHashCellSize = cellSize
end

function World:_getNearbyPrimitives(x, y, z, radius)
	if not self.collisionHash then self:_rebuildCollisionHash() end
	if not self.collisionHash then return {} end
	local rawResults = querySpatialHash(self.collisionHash, x, y, z, radius or 5, self.collisionHashCellSize or 4)
	-- Wrap raw primitives back into {type, prim} format for collision function
	local results = {}
	for _, raw in rawResults do
		-- Determine type from raw primitive
		if raw.radius then
			table.insert(results, { type = "sphere", prim = raw })
		else
			table.insert(results, { type = "box", prim = raw })
		end
	end
	return results
end
function World:_recordStaticImpact(n, normal, incomingNormal, penetration, restitution)
	if incomingNormal >= 0 then return end
	local impulseMagnitude = -incomingNormal * n.mass * (1 + restitution)
	n.impactImpulseX = (n.impactImpulseX or 0) + normal[1] * impulseMagnitude
	n.impactImpulseY = (n.impactImpulseY or 0) + normal[2] * impulseMagnitude
	n.impactImpulseZ = (n.impactImpulseZ or 0) + normal[3] * impulseMagnitude

	local centerX, centerY, centerZ, totalMass, dynamicCount = 0, 0, 0, 0, 0
	for _, candidate in self.nodes do
		if not candidate.fixed then
			centerX += candidate.x
			centerY += candidate.y
			centerZ += candidate.z
			totalMass += candidate.mass
			dynamicCount += 1
		end
	end
	if dynamicCount > 0 then
		centerX /= dynamicCount
		centerY /= dynamicCount
		centerZ /= dynamicCount
	end

	local severity = math.clamp(impulseMagnitude / math.max(totalMass * 8, 1), 0, 1)
	local telemetry = self.telemetry
	telemetry[Contract.T.IMPACT_SEVERITY + 1] = math.max(telemetry[Contract.T.IMPACT_SEVERITY + 1], severity)
	telemetry[Contract.T.DAMAGE + 1] = math.clamp(telemetry[Contract.T.DAMAGE + 1] + severity * 0.4, 0, 1)
	telemetry[Contract.T.DEFORMATION_DEPTH + 1] = math.max(telemetry[Contract.T.DEFORMATION_DEPTH + 1], penetration)
	telemetry[Contract.T.DAMAGE_ZONE + 1] = Math.classifyDamageZone(n.x, n.y, n.z, centerX, centerZ)
end

-- Terrain collision
function World:_collideWithTerrain()
	local nodes = self.nodes
	local count = #nodes
	if count == 0 then return end
	-- Decay impact severity
	self.telemetry[Contract.T.IMPACT_SEVERITY + 1] *= 0.90 ^ (self.fixedDt * 60)
	-- Compute center
	local centerX, centerZ, totalMass = 0, 0, 0
	local dynCount = 0
	for i = 1, count do
		if not nodes[i].fixed then
			centerX += nodes[i].x
			centerZ += nodes[i].z
			dynCount += 1
		end
		totalMass += nodes[i].mass
	end
	totalMass = math.max(totalMass, 1)
	if dynCount > 0 then
		centerX /= dynCount
		centerZ /= dynCount
	end

	for i = 1, count do
		local n = nodes[i]
		if not n.fixed and n.collision then
			local th = self:terrainHeight(n.x, n.z)
			local height = th
			if i > 4 then height += 0.2 end -- body node clearance
			if n.y < height then
				local wasBelowTerrain = n.y < th
				local normal = self:terrainNormal(n.x, n.z)
				local nv = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
				if nv < 0 then
					local impulse = -nv * n.mass
					if impulse > totalMass * 0.25 then
						local severity = math.clamp(impulse / (totalMass * 8), 0, 1)
						self.telemetry[Contract.T.IMPACT_SEVERITY + 1] = math.max(self.telemetry[Contract.T.IMPACT_SEVERITY + 1], severity)
						self.telemetry[Contract.T.DAMAGE + 1] = math.clamp(self.telemetry[Contract.T.DAMAGE + 1] + severity * 0.03, 0, 1)
						-- Classify damage zone
						self.telemetry[Contract.T.DAMAGE_ZONE + 1] = Math.classifyDamageZone(n.x, n.y, n.z, centerX, centerZ)
					end
					local restitution = 0.15
					n.vx -= nv * (1 + restitution) * normal[1]
					n.vy -= nv * (1 + restitution) * normal[2]
					n.vz -= nv * (1 + restitution) * normal[3]
				end
				n.y = height
				-- Coulomb friction on per-node ground contact
				-- Normal force from gravity component + dynamic impulse
				if wasBelowTerrain then
					local tangentX = n.vx - n.vy * normal[1]
					local tangentZ = n.vz - n.vy * normal[3]
					local tangentSpeed = math.sqrt(tangentX * tangentX + tangentZ * tangentZ)
					if tangentSpeed > 1e-6 and n.mass > 0 then
						-- Coulomb: F = μ × N, Δv = F/m × dt
						-- N includes weight component along normal + dynamic impulse
						local weightNormal = math.abs(self.gravity) * n.mass * math.abs(normal[2])
						local dynamicNormal = math.abs(nv) * n.mass * (1 + restitution)
						local maxDecel = self.groundFriction * (weightNormal + dynamicNormal) / n.mass
						local factor = math.clamp(1 - maxDecel * self.fixedDt / tangentSpeed, 0, 1)
						n.vx = tangentX * factor
						n.vz = tangentZ * factor
					end
				end
			end
		end
	end
end

-- Static geometry collision
function World:_collideWithStaticGeometry()
	local nodes = self.nodes
	local count = #nodes
	for _, n in nodes do
		n.impactImpulseX = 0
		n.impactImpulseY = 0
		n.impactImpulseZ = 0
	end

	-- Build spatial hash if needed (guard for minimal world objects)
	if self._rebuildCollisionHash and not self.collisionHash then
		self:_rebuildCollisionHash()
	end

	for i = 1, count do
		local n = nodes[i]
		-- Wheel/suspension nodes may skip analytic terrain collision, but they
		-- still need to collide with authored walls and other static geometry.
		local canCollideStatic = if n.staticCollision == nil then n.collision else n.staticCollision
		if n.fixed or not canCollideStatic then continue end

		-- Broadphase: query nearby primitives (fallback to full list if spatial hash unavailable)
		local nearby
		if self._getNearbyPrimitives then
			nearby = self:_getNearbyPrimitives(n.x, n.y, n.z, 5)
		else
			-- Fallback: check all primitives
			nearby = {}
			for _, box in self.collisionBoxes do table.insert(nearby, { type = "box", prim = box }) end
			for _, sphere in self.collisionSpheres do table.insert(nearby, { type = "sphere", prim = sphere }) end
			if self.mapCollisionBoxes then
				for _, box in self.mapCollisionBoxes do table.insert(nearby, { type = "box", prim = box }) end
			end
			if self.mapCollisionSpheres then
				for _, sphere in self.mapCollisionSpheres do table.insert(nearby, { type = "sphere", prim = sphere }) end
			end
		end

		for _, entry in nearby do
			if entry.type == "box" then
				local box = entry.prim
				local rx, ry, rz = n.x - box.center[1], n.y - box.center[2], n.z - box.center[3]
				local basis = box.basis
				local ax1, ax2, ax3 = 1, 0, 0
				local ay1, ay2, ay3 = 0, 1, 0
				local az1, az2, az3 = 0, 0, 1
				if basis then
					ax1, ax2, ax3 = basis[1][1], basis[1][2], basis[1][3]
					ay1, ay2, ay3 = basis[2][1], basis[2][2], basis[2][3]
					az1, az2, az3 = basis[3][1], basis[3][2], basis[3][3]
				end
				local lx = rx * ax1 + ry * ax2 + rz * ax3
				local ly = rx * ay1 + ry * ay2 + rz * ay3
				local lz = rx * az1 + ry * az2 + rz * az3
				local px, py, pz = box.halfExtents[1] - math.abs(lx), box.halfExtents[2] - math.abs(ly), box.halfExtents[3] - math.abs(lz)
				if px > 0 and py > 0 and pz > 0 then
					-- Find min penetration axis
					local axis = 1
					local minPen = px
					if py < minPen then axis = 2; minPen = py end
					if pz < minPen then axis = 3; minPen = pz end
					local localNormalX, localNormalY, localNormalZ = 0, 0, 0
					local localVelocityX = n.vx * ax1 + n.vy * ax2 + n.vz * ax3
					local localVelocityY = n.vx * ay1 + n.vy * ay2 + n.vz * ay3
					local localVelocityZ = n.vx * az1 + n.vy * az2 + n.vz * az3
					if axis == 1 then
						localNormalX = if math.abs(lx) > 1e-8 then Math.sign(lx) else -Math.sign(localVelocityX)
					elseif axis == 2 then
						localNormalY = if math.abs(ly) > 1e-8 then Math.sign(ly) else -Math.sign(localVelocityY)
					else
						localNormalZ = if math.abs(lz) > 1e-8 then Math.sign(lz) else -Math.sign(localVelocityZ)
					end
					if localNormalX == 0 and localNormalY == 0 and localNormalZ == 0 then localNormalX = 1 end
					local normal = {
						localNormalX * ax1 + localNormalY * ay1 + localNormalZ * az1,
						localNormalX * ax2 + localNormalY * ay2 + localNormalZ * az2,
						localNormalX * ax3 + localNormalY * ay3 + localNormalZ * az3,
					}
					local incomingNormal = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
					self:_recordStaticImpact(n, normal, incomingNormal, minPen, box.restitution)
					local corr = minPen
					n.x += normal[1] * corr
					n.y += normal[2] * corr
					n.z += normal[3] * corr
					self:_applyContactVelocity(n, normal, box.restitution, box.friction)
				end
			elseif entry.type == "sphere" then
				local sphere = entry.prim
				local rx, ry, rz = n.x - sphere.center[1], n.y - sphere.center[2], n.z - sphere.center[3]
				local dist = math.sqrt(rx*rx + ry*ry + rz*rz)
				if dist < sphere.radius then
					local normal
					if dist > 1e-8 then
						normal = {rx/dist, ry/dist, rz/dist}
					else
						normal = {0, 1, 0}
					end
					local incomingNormal = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
					self:_recordStaticImpact(n, normal, incomingNormal, sphere.radius - dist, sphere.restitution)
					local corr = sphere.radius - dist
					n.x += normal[1] * corr
					n.y += normal[2] * corr
					n.z += normal[3] * corr
					self:_applyContactVelocity(n, normal, sphere.restitution, sphere.friction)
				end
			end
		end

		-- Boundaries (no spatial hash, typically few)
		for _, boundary in self.boundaries do
			local dx = n.x - boundary.point[1]
			local dy = n.y - boundary.point[2]
			local dz = n.z - boundary.point[3]
			local normal = boundary.normal or { 0, 1, 0 }
			local signedDistance = dx * normal[1] + dy * normal[2] + dz * normal[3]
			if signedDistance < 0 then
				local incomingNormal = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
				self:_recordStaticImpact(n, normal, incomingNormal, -signedDistance, boundary.restitution)
				local corr = -signedDistance
				n.x += normal[1] * corr
				n.y += normal[2] * corr
				n.z += normal[3] * corr
				self:_applyContactVelocity(n, normal, boundary.restitution, boundary.friction)
			end
		end
	end
end

-- ============================================================
-- Map object collision registration
-- ============================================================

-- Batch-register collision primitives from map assets.
-- Objects is an array of tables with shape: { type, position, size, rotation, restitution, friction }
-- Supported types: "box", "sphere"
-- Map objects use separate tables so they can be cleared on map transitions.
function World:addMapObjects(objects)
	if type(objects) ~= "table" then return end
	if not self.mapCollisionBoxes then self.mapCollisionBoxes = {} end
	if not self.mapCollisionSpheres then self.mapCollisionSpheres = {} end

	for _, obj in objects do
		if type(obj) ~= "table" then continue end
		local objType = obj.type
		local pos = obj.position
		if type(pos) ~= "table" or #pos < 3 then continue end
		local cx, cy, cz = pos[1], pos[2], pos[3]
		local rest = math.clamp(obj.restitution or 0.3, 0, 1)
		local fric = math.clamp(obj.friction or 0.7, 0, 2)

		if objType == "box" then
			local size = obj.size
			if type(size) ~= "table" or #size < 3 then continue end
			local hx, hy, hz = size[1] * 0.5, size[2] * 0.5, size[3] * 0.5
			local basis = nil
			if obj.rotation and type(obj.rotation) == "table" and #obj.rotation == 3 then
				basis = obj.rotation
			end
			table.insert(self.mapCollisionBoxes, {
				center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
				halfExtents = {
					math.max(math.abs(hx), 1e-4),
					math.max(math.abs(hy), 1e-4),
					math.max(math.abs(hz), 1e-4),
				},
				restitution = rest,
				friction = fric,
				basis = basis,
			})
		elseif objType == "sphere" then
			local radius = obj.size
			if type(radius) == "table" then radius = radius[1] end
			if type(radius) ~= "number" or radius <= 0 then continue end
			table.insert(self.mapCollisionSpheres, {
				center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
				radius = math.max(math.abs(radius), 1e-4),
				restitution = rest,
				friction = fric,
			})
		end
	end
end

-- Remove all map-specific collision primitives (call on map transition).
function World:clearMapObjects()
	if self.mapCollisionBoxes then table.clear(self.mapCollisionBoxes) end
	if self.mapCollisionSpheres then table.clear(self.mapCollisionSpheres) end
end

function World:_applyContactVelocity(n, normal, restitution, friction)
	local nv = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
	if nv < 0 then
		n.vx -= nv * (1 + restitution) * normal[1]
		n.vy -= nv * (1 + restitution) * normal[2]
		n.vz -= nv * (1 + restitution) * normal[3]
	end
	-- Coulomb friction: decelerate tangential velocity by μ × |normal_impulse| / mass.
	-- normal_impulse ≈ m × |nv| × (1+e), so Δv_friction ≤ μ × |nv| × (1+e).
	local postN = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
	local tvx = n.vx - postN * normal[1]
	local tvy = n.vy - postN * normal[2]
	local tvz = n.vz - postN * normal[3]
	local tangentialSpeed = math.sqrt(tvx*tvx + tvy*tvy + tvz*tvz)
	if tangentialSpeed > 1e-6 then
		local maxDecel = friction * math.abs(nv) * (1 + restitution)
		local factor = math.clamp(1 - maxDecel / tangentialSpeed, 0, 1)
		n.vx = postN * normal[1] + tvx * factor
		n.vy = postN * normal[2] + tvy * factor
		n.vz = postN * normal[3] + tvz * factor
	end
end
end

return Module
