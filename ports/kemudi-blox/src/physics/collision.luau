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
				-- Ground friction on penetrating body
				if wasBelowTerrain then
					n.vx *= self.groundFriction
					n.vz *= self.groundFriction
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
		if n.fixed or (not n.collision) then continue end
		-- Boxes
		for _, box in self.collisionBoxes do
			local rx, ry, rz = n.x - box.center[1], n.y - box.center[2], n.z - box.center[3]
			local px, py, pz = box.halfExtents[1] - math.abs(rx), box.halfExtents[2] - math.abs(ry), box.halfExtents[3] - math.abs(rz)
			if px > 0 and py > 0 and pz > 0 then
				-- Find min penetration axis
				local axis = 1
				local minPen = px
				if py < minPen then axis = 2; minPen = py end
				if pz < minPen then axis = 3; minPen = pz end
				local normal = {0, 0, 0}
				local rel = {rx, ry, rz}
				local vel = {n.vx, n.vy, n.vz}
				if math.abs(rel[axis]) > 1e-8 then
					normal[axis] = Math.sign(rel[axis])
				elseif math.abs(vel[axis]) > 1e-8 then
					normal[axis] = -Math.sign(vel[axis])
				else
					normal[axis] = 1
				end
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
	local ts = math.clamp(1 - friction * 0.25, 0, 1)
	local postN = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
	n.vx = (n.vx - postN * normal[1]) * ts + postN * normal[1]
	n.vy = (n.vy - postN * normal[2]) * ts + postN * normal[2]
	n.vz = (n.vz - postN * normal[3]) * ts + postN * normal[3]
end
end

return Module
