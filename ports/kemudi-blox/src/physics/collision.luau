local Module = {}

function Module.install(World, Contract, Math)
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
	for i = 1, count do
		local n = nodes[i]
		-- Wheel/suspension nodes may skip analytic terrain collision, but they
		-- still need to collide with authored walls and other static geometry.
		local canCollideStatic = if n.staticCollision == nil then n.collision else n.staticCollision
		if n.fixed or not canCollideStatic then continue end
		-- Boxes
		for _, box in self.collisionBoxes do
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
				local corr = math.min(minPen, 0.01)
				n.x += normal[1] * corr
				n.y += normal[2] * corr
				n.z += normal[3] * corr
				self:_applyContactVelocity(n, normal, box.restitution, box.friction)
			end
		end
		-- Spheres
		for _, sphere in self.collisionSpheres do
			local rx, ry, rz = n.x - sphere.center[1], n.y - sphere.center[2], n.z - sphere.center[3]
			local dist = math.sqrt(rx*rx + ry*ry + rz*rz)
			if dist < sphere.radius then
				local normal
				if dist > 1e-8 then
					normal = {rx/dist, ry/dist, rz/dist}
				else
					normal = {0, 1, 0}
				end
				local corr = math.min(sphere.radius - dist, 0.01)
				n.x += normal[1] * corr
				n.y += normal[2] * corr
				n.z += normal[3] * corr
				self:_applyContactVelocity(n, normal, sphere.restitution, sphere.friction)
			end
		end
		-- Boundaries
		for _, boundary in self.boundaries do
			local dx = n.x - boundary.point[1]
			local dy = n.y - boundary.point[2]
			local dz = n.z - boundary.point[3]
			local normal = boundary.normal or { 0, 1, 0 }
			local signedDistance = dx * normal[1] + dy * normal[2] + dz * normal[3]
			if signedDistance < 0 then
				local corr = math.min(-signedDistance, 0.05)
				n.x += normal[1] * corr
				n.y += normal[2] * corr
				n.z += normal[3] * corr
				self:_applyContactVelocity(n, normal, boundary.restitution, boundary.friction)
			end
		end
	end
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
