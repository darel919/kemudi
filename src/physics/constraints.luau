local Module = {}

local function dot3(ax, ay, az, bx, by, bz)
	return ax * bx + ay * by + az * bz
end

local function normalize3(x, y, z)
	local length = math.sqrt(x * x + y * y + z * z)
	if length <= 1e-8 or length ~= length then return nil end
	return x / length, y / length, z / length
end

local function cross3(ax, ay, az, bx, by, bz)
	return ay * bz - az * by,
		az * bx - ax * bz,
		ax * by - ay * bx
end

local function scale3(x, y, z, s)
	return x * s, y * s, z * s
end

local function chassisBasis(points)
	local frontX = (points[1][1] + points[2][1]) * 0.5
	local frontY = (points[1][2] + points[2][2]) * 0.5
	local frontZ = (points[1][3] + points[2][3]) * 0.5
	local rearX = (points[3][1] + points[4][1]) * 0.5
	local rearY = (points[3][2] + points[4][2]) * 0.5
	local rearZ = (points[3][3] + points[4][3]) * 0.5
	local leftX = (points[1][1] + points[3][1]) * 0.5
	local leftY = (points[1][2] + points[3][2]) * 0.5
	local leftZ = (points[1][3] + points[3][3]) * 0.5
	local rightX = (points[2][1] + points[4][1]) * 0.5
	local rightY = (points[2][2] + points[4][2]) * 0.5
	local rightZ = (points[2][3] + points[4][3]) * 0.5
	local fx, fy, fz = normalize3(frontX - rearX, frontY - rearY, frontZ - rearZ)
	if not fx then return nil end
	local rawX, rawY, rawZ = rightX - leftX, rightY - leftY, rightZ - leftZ
	local projection = dot3(rawX, rawY, rawZ, fx, fy, fz)
	local rx, ry, rz = normalize3(rawX - fx * projection, rawY - fy * projection, rawZ - fz * projection)
	if not rx then return nil end
	local ux, uy, uz = normalize3(ry * fz - rz * fy, rz * fx - rx * fz, rx * fy - ry * fx)
	if not ux then return nil end
	return { { rx, ry, rz }, { ux, uy, uz }, { fx, fy, fz } }
end

function Module.install(World, Math)
-- XPBD beam and triangle constraint projection.
function World:_solveXPBD()
	local ITERATIONS = 30
	local dt = self.fixedDt
	if not (dt > 0 and dt == dt) then dt = 1 / 60 end
	local dt2 = dt * dt
	local nodes = self.nodes
	local beams = self.beams
	local triangles = self.triangles

	-- Reset lambdas
	for i = 1, #beams do beams[i].lambda = 0 end
	for i = 1, #triangles do triangles[i].lambda = 0 end
	self.volumeLambda = 0

	for _iter = 1, ITERATIONS do
		-- Beam projection
		for bi = 1, #beams do
			local beam = beams[bi]
			if not beam.broken then
				local a = beam.nodeA + 1
				local b = beam.nodeB + 1
				if a <= #nodes and b <= #nodes then
					local na = nodes[a]
					local nb = nodes[b]
					local dx = nb.x - na.x
					local dy = nb.y - na.y
					local dz = nb.z - na.z
					local len = math.sqrt(dx*dx + dy*dy + dz*dz)
					if len > 1e-8 then
						local c = len - beam.length
						local invSum = na.invMass + nb.invMass
						if invSum > 0 then
							local compliance = 1 / math.max(beam.stiffness, 1)
							local alpha = compliance / dt2
							local dLambda = (-c - alpha * beam.lambda) / (invSum + alpha)
							beam.lambda += dLambda
							local nx, ny, nz = dx/len, dy/len, dz/len
							if not na.fixed then
								na.x -= dLambda * na.invMass * nx
								na.y -= dLambda * na.invMass * ny
								na.z -= dLambda * na.invMass * nz
							end
							if not nb.fixed then
								nb.x += dLambda * nb.invMass * nx
								nb.y += dLambda * nb.invMass * ny
								nb.z += dLambda * nb.invMass * nz
							end
						end
					end
				end
			end
		end

		-- Triangle area preservation
		for ti = 1, #triangles do
			local tri = triangles[ti]
			local ai, bi2, ci = tri.a + 1, tri.b + 1, tri.c + 1
			if ai <= #nodes and bi2 <= #nodes and ci <= #nodes then
						local a = nodes[ai]
						local b = nodes[bi2]
						local c = nodes[ci]
						local abx, aby, abz = b.x - a.x, b.y - a.y, b.z - a.z
						local acx, acy, acz = c.x - a.x, c.y - a.y, c.z - a.z
						local crx, cry, crz = cross3(abx, aby, abz, acx, acy, acz)
						local doubleArea = math.sqrt(crx*crx + cry*cry + crz*crz)
						if doubleArea >= 1e-10 then
							local nn = { crx/doubleArea, cry/doubleArea, crz/doubleArea }
							-- Gradients of 0.5 * |ab x ac|
							local gAx, gAy, gAz = cross3(b.x-c.x, b.y-c.y, b.z-c.z, nn[1], nn[2], nn[3])
							local gradAx, gradAy, gradAz = scale3(gAx, gAy, gAz, 0.5)
							local gBx, gBy, gBz = cross3(c.x-a.x, c.y-a.y, c.z-a.z, nn[1], nn[2], nn[3])
							local gradBx, gradBy, gradBz = scale3(gBx, gBy, gBz, 0.5)
							local gCx, gCy, gCz = cross3(nn[1], nn[2], nn[3], abx, aby, abz)
							local gradCx, gradCy, gradCz = scale3(gCx, gCy, gCz, 0.5)
							local area = doubleArea * 0.5
							local error_ = area - tri.restArea
							local compliance = 1e-5 / math.max(dt2, 1e-12)
							local wSum = a.invMass * (gradAx*gradAx+gradAy*gradAy+gradAz*gradAz)
								+ b.invMass * (gradBx*gradBx+gradBy*gradBy+gradBz*gradBz)
								+ c.invMass * (gradCx*gradCx+gradCy*gradCy+gradCz*gradCz)
							local denom = wSum + compliance
							if denom > 1e-12 then
								local dL = (-error_ - compliance * tri.lambda) / denom
								tri.lambda += dL
								if a.invMass > 0 then
									a.x += dL * a.invMass * gradAx
									a.y += dL * a.invMass * gradAy
									a.z += dL * a.invMass * gradAz
								end
						if b.invMass > 0 then
							b.x += dL * b.invMass * gradBx
							b.y += dL * b.invMass * gradBy
							b.z += dL * b.invMass * gradBz
						end
						if c.invMass > 0 then
							c.x += dL * c.invMass * gradCx
							c.y += dL * c.invMass * gradCy
							c.z += dL * c.invMass * gradCz
						end
					end
				end
			end
		end
		self:_solveBodyAttachmentConstraints()
		self:_solveVolumePreservation()
	end

	-- Beam damping (once after iterations)
	self:_applyBeamDamping(dt)

	-- Feed constraint correction into velocity
	local invDt = 1 / math.max(dt, 1e-6)
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			local sp = self.constraintStartPositions[i]
			if sp then
				n.vx += (n.x - sp[1]) * invDt
				n.vy += (n.y - sp[2]) * invDt
				n.vz += (n.z - sp[3]) * invDt
			end
		end
	end
end

-- Preserve the authored upper cage in the current four-mount chassis frame.
-- Distance beams retain length but do not prevent a body from shearing or
-- yawing as an unconstrained linkage. Broken direct beams release the mount.
function World:_solveBodyAttachmentConstraints()
	local nodeCount = math.min(#self.nodes, #self.restPositions)
	if nodeCount < 8 then return end
	local currentMounts = {
		{ self.nodes[1].x, self.nodes[1].y, self.nodes[1].z },
		{ self.nodes[2].x, self.nodes[2].y, self.nodes[2].z },
		{ self.nodes[3].x, self.nodes[3].y, self.nodes[3].z },
		{ self.nodes[4].x, self.nodes[4].y, self.nodes[4].z },
	}
	local restMounts = {
		self.restPositions[1], self.restPositions[2],
		self.restPositions[3], self.restPositions[4],
	}
	local currentBasis = chassisBasis(currentMounts)
	local restBasis = chassisBasis(restMounts)
	if not currentBasis or not restBasis then return end

	local upperCount = math.min(12, nodeCount)
	for index = 5, upperCount do
		local reference = if index <= 8 then index - 4 else 5
		if index > 8 then
			local nearestDistance = math.huge
			for candidate = 5, math.min(8, nodeCount) do
				local dx = self.restPositions[index][1] - self.restPositions[candidate][1]
				local dz = self.restPositions[index][3] - self.restPositions[candidate][3]
				local distance = dx * dx + dz * dz
				if distance < nearestDistance then
					reference, nearestDistance = candidate, distance
				end
			end
		end

		local broken = false
		for _, beam in self.beams do
			if beam.broken and ((beam.nodeA == reference - 1 and beam.nodeB == index - 1)
				or (beam.nodeA == index - 1 and beam.nodeB == reference - 1)) then
				broken = true
				break
			end
		end
		if not broken then
			local upper = self.nodes[index]
			local mount = self.nodes[reference]
			local restUpper = self.restPositions[index]
			local restMount = self.restPositions[reference]
			local dx = restUpper[1] - restMount[1]
			local dy = restUpper[2] - restMount[2]
			local dz = restUpper[3] - restMount[3]
			local localX = dot3(dx, dy, dz, restBasis[1][1], restBasis[1][2], restBasis[1][3])
			local localY = dot3(dx, dy, dz, restBasis[2][1], restBasis[2][2], restBasis[2][3])
			local localZ = dot3(dx, dy, dz, restBasis[3][1], restBasis[3][2], restBasis[3][3])
			local expectedX = currentBasis[1][1] * localX + currentBasis[2][1] * localY + currentBasis[3][1] * localZ
			local expectedY = currentBasis[1][2] * localX + currentBasis[2][2] * localY + currentBasis[3][2] * localZ
			local expectedZ = currentBasis[1][3] * localX + currentBasis[2][3] * localY + currentBasis[3][3] * localZ
			local errorX = upper.x - mount.x - expectedX
			local errorY = upper.y - mount.y - expectedY
			local errorZ = upper.z - mount.z - expectedZ
			local invMassSum = upper.invMass + mount.invMass
			if invMassSum > 0 then
			local softness = self.bodyAttachmentSoftness
			if not softness then return end
				local correctionX = -errorX / invMassSum * softness
				local correctionY = -errorY / invMassSum * softness
				local correctionZ = -errorZ / invMassSum * softness
				if not mount.fixed then
					mount.x -= correctionX * mount.invMass
					mount.y -= correctionY * mount.invMass
					mount.z -= correctionZ * mount.invMass
				end
				if not upper.fixed then
					upper.x += correctionX * upper.invMass
					upper.y += correctionY * upper.invMass
					upper.z += correctionZ * upper.invMass
				end
			end
		end
	end
end

function World:_applyBeamDamping(dt)
	local beams = self.beams
	local nodes = self.nodes
	for bi = 1, #beams do
		local beam = beams[bi]
		if not beam.broken then
			local ai = beam.nodeA + 1
			local bi2 = beam.nodeB + 1
			if ai <= #nodes and bi2 <= #nodes then
				local a = nodes[ai]
				local b = nodes[bi2]
				local dx = b.x - a.x
				local dy = b.y - a.y
				local dz = b.z - a.z
				local len = math.sqrt(dx*dx + dy*dy + dz*dz)
				if len > 1e-8 then
					local nx, ny, nz = dx/len, dy/len, dz/len
					local relV = (b.vx - a.vx)*nx + (b.vy - a.vy)*ny + (b.vz - a.vz)*nz
					local damping = math.max(beam.damping, 0)
					if damping > 0 then
						local effMass = a.invMass + b.invMass
						local impulse = damping * relV * dt / (1 + damping * dt * effMass)
						if not a.fixed then
							a.vx += impulse * a.invMass * nx
							a.vy += impulse * a.invMass * ny
							a.vz += impulse * a.invMass * nz
						end
						if not b.fixed then
							b.vx -= impulse * b.invMass * nx
							b.vy -= impulse * b.invMass * ny
							b.vz -= impulse * b.invMass * nz
						end
					end
				end
			end
		end
	end
end

function World:_updateBeamMaterialState(dt)
	local nodes = self.nodes
	local beams = self.beams
	local positions = {}
	for i = 1, #nodes do
		local n = nodes[i]
		positions[i] = { n.x, n.y, n.z }
	end
	local dt2 = dt * dt
	for bi = 1, #beams do
		local beam = beams[bi]
		local ai = beam.nodeA + 1
		local bi2 = beam.nodeB + 1
		if ai <= #positions and bi2 <= #positions then
			local a = positions[ai]
			local b = positions[bi2]
			local dx, dy, dz = b[1]-a[1], b[2]-a[2], b[3]-a[3]
			local curLen = math.sqrt(dx*dx + dy*dy + dz*dz)
			if curLen > 1e-8 then
				local solverLoad = math.abs(beam.lambda / math.max(dt2, 1e-12))
				local nx, ny, nz = dx / curLen, dy / curLen, dz / curLen
				local aNode, bNode = nodes[ai], nodes[bi2]
				local aImpulseAlong = (aNode.impactImpulseX or 0) * nx
					+ (aNode.impactImpulseY or 0) * ny
					+ (aNode.impactImpulseZ or 0) * nz
				local bImpulseAlong = (bNode.impactImpulseX or 0) * nx
					+ (bNode.impactImpulseY or 0) * ny
					+ (bNode.impactImpulseZ or 0) * nz
				local contactTension = math.max(-aImpulseAlong, bImpulseAlong) / math.max(dt, 1e-6)
				local axialLoad = math.max(solverLoad, contactTension)
				beam.stress = axialLoad
				local tensile = math.max(if beam.lambda < 0 then solverLoad else 0, contactTension)
				if tensile > beam.strength then
					beam.broken = true
				elseif tensile > beam.yieldStrength and beam.plasticity > 0 then
					local excess = (tensile - beam.yieldStrength) / math.max(beam.stiffness, 1)
					local inc = math.clamp(excess * beam.plasticity, 0, beam.initialLength * 0.02)
					beam.length = math.clamp(beam.length + inc, 1e-4, beam.initialLength * 1.4)
				end
			end
		end
	end
end

-- XPBD volume preservation constraint.
-- Maintains the signed mesh volume using a triangle fan around the rest centroid.
-- Prevents unrealistic flat-collapse while allowing deformation.
function World:_solveVolumePreservation()
	local triangles = self.triangles
	local nodes = self.nodes
	local nodeCount = #nodes
	if #triangles < 4 or self.restVolume <= 0 then return end

	local dt = self.fixedDt
	local dt2 = dt * dt
	local px, py, pz = self.restCentroid[1], self.restCentroid[2], self.restCentroid[3]

	-- Accumulate gradients per node and compute current volume
	local gx = table.create(nodeCount, 0)
	local gy = table.create(nodeCount, 0)
	local gz = table.create(nodeCount, 0)
	local currentVolume = 0

	for ti = 1, #triangles do
		local tri = triangles[ti]
		local ai, bi, ci = tri.a + 1, tri.b + 1, tri.c + 1
		if ai <= nodeCount and bi <= nodeCount and ci <= nodeCount then
			local na, nb, nc = nodes[ai], nodes[bi], nodes[ci]
			local ax, ay, az = na.x - px, na.y - py, na.z - pz
			local bx, by, bz = nb.x - px, nb.y - py, nb.z - pz
			local cx, cy, cz = nc.x - px, nc.y - py, nc.z - pz

			-- Signed volume contribution of this tetrahedron
			currentVolume += (ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx)) / 6

			-- Accumulated gradients: grad_a = (1/6)(b-P)×(c-P), etc.
			gx[ai] += (by * cz - bz * cy) / 6
			gy[ai] += (bz * cx - bx * cz) / 6
			gz[ai] += (bx * cy - by * cx) / 6

			gx[bi] += (cy * az - cz * ay) / 6
			gy[bi] += (cz * ax - cx * az) / 6
			gz[bi] += (cx * ay - cy * ax) / 6

			gx[ci] += (ay * bz - az * by) / 6
			gy[ci] += (az * bx - ax * bz) / 6
			gz[ci] += (ax * by - ay * bx) / 6
		end
	end

	-- Weighted denominator
	local wSum = 0
	for i = 1, nodeCount do
		local n = nodes[i]
		if n.invMass > 0 and not n.fixed then
			wSum += n.invMass * (gx[i] * gx[i] + gy[i] * gy[i] + gz[i] * gz[i])
		end
	end

	local compliance = self.volumeCompliance / math.max(dt2, 1e-12)
	local denom = wSum + compliance
	if denom <= 1e-12 then return end

	local error_ = currentVolume - self.restVolume
	local dL = (-error_ - compliance * self.volumeLambda) / denom
	self.volumeLambda += dL

	-- Apply accumulated corrections
	for i = 1, nodeCount do
		local n = nodes[i]
		if n.invMass > 0 and not n.fixed then
			n.x += dL * n.invMass * gx[i]
			n.y += dL * n.invMass * gy[i]
			n.z += dL * n.invMass * gz[i]
		end
	end
end

end

return Module
