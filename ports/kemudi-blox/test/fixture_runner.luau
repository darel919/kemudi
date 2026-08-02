--[[
	Generic fixture runner for the Kemudi physics engine.
	Loads any fixture JSON from fixtures/engine/, replays steps, checks assertions.
	Run with: lune run test/fixture_runner.luau [fixture_name]
	Without args: runs all fixtures in fixtures/engine/.
--]]

local fs = require("@lune/fs")
local serde = require("@lune/serde")
local process = require("@lune/process")

-- ============================================================
-- Inlined physics engine (same as gravity_fixture_test.luau)
-- ============================================================

local T = {
	SPEED_MPS = 0, SPEED_KMH = 1, SIM_TIME = 78, BROKEN_BEAMS = 85,
	DAMAGE = 83, IMPACT_SEVERITY = 80, DAMAGE_ZONE = 84, DERATE = 29,
}
local TELEMETRY_LEN = 86
local GRAVITY = -9.81

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

local function finite_or_zero(v)
	if v ~= v then return 0 end
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

local function sign(v)
	if v > 0 then return 1 end
	if v < 0 then return -1 end
	return 0
end

local function cross3(a, b)
	return {
		a[2]*b[3] - a[3]*b[2],
		a[3]*b[1] - a[1]*b[3],
		a[1]*b[2] - a[2]*b[1],
	}
end

local function scale3(v, s)
	return { v[1]*s, v[2]*s, v[3]*s }
end

local function dist3(a, b)
	local dx, dy, dz = b[1]-a[1], b[2]-a[2], b[3]-a[3]
	return math.sqrt(dx*dx + dy*dy + dz*dz)
end

local function triangleArea(ax, ay, az, bx, by, bz, cx, cy, cz)
	local abx, aby, abz = bx-ax, by-ay, bz-az
	local acx, acy, acz = cx-ax, cy-ay, cz-az
	local cr = cross3({abx,aby,abz},{acx,acy,acz})
	return math.sqrt(cr[1]*cr[1] + cr[2]*cr[2] + cr[3]*cr[3]) * 0.5
end

local function classifyDamageZone(x, y, z, cx, cz)
	local dx, dz = x - cx, z - cz
	if math.abs(dx) > math.abs(dz) then
		return if dx > 0 then 3 else 2  -- left/right
	else
		return if dz > 0 then 1 else 0  -- front/rear
	end
end

-- ============================================================
-- World (full copy)
-- ============================================================

local World = {}
World.__index = World

function World.new()
	local self = setmetatable({}, World)
	self.nodes = {}
	self.beams = {}
	self.triangles = {}
	self.gravity = GRAVITY
	self.time = 0
	self.airDensity = 1.225
	self.dragCoefficient = 0.5
	self.frontalArea = 2.2
	self.accumulator = 0
	self.fixedDt = 1/60
	self.maxSubsteps = 8
	self.restPositions = {}
	self.baseNodeMasses = {}
	self.constraintStartPositions = {}
	self.controls = { steering = 0, throttle = 0, brake = 0, clutch = 0,
		handbrake = false, gearUp = false, gearDown = false, engineOn = false }
	self.terrainProfile = 0
	self.terrainRuts = {}
	self.terrainHeights = {}
	self.terrainWidth = 0
	self.terrainDepth = 0
	self.terrainSegments = 0
	self.groundFriction = 0.94
	self.collisionBoxes = {}
	self.collisionSpheres = {}
	self.boundaries = {}
	self.roadSurfaces = {}
	self.telemetry = table.create(TELEMETRY_LEN, 0)
	self.telemetry[T.DERATE + 1] = 1.0
	self.suspension_config = nil
	self.suspension_states = nil
	self.steering_config = nil
	self.steering_angle = 0
	self.tires = nil
	self.tire_thermal = nil
	self.drivetrain = nil
	self.safety = nil
	self.fuel_tank = nil
	self.vehicle_mass = 0
	return self
end

function World:setupVehicle() end

function World:addNode(id, x, y, z, mass, fixed, collision)
	x = finite_or_zero(x); y = finite_or_zero(y); z = finite_or_zero(z)
	if not (mass > 0 and mass == mass) then mass = 1 end
	local invMass = if fixed then 0 else 1 / mass
	local c = #self.nodes
	local coll = collision
	if coll == nil then
		coll = (not fixed) and c >= 4
	end
	table.insert(self.nodes, {
		id = id, x = x, y = y, z = z,
		vx = 0, vy = 0, vz = 0,
		mass = mass, invMass = invMass, fixed = fixed,
		collision = coll,
		fx = 0, fy = 0, fz = 0, lastForce = 0,
	})
	table.insert(self.constraintStartPositions, {x, y, z})
	table.insert(self.restPositions, {x, y, z})
	table.insert(self.baseNodeMasses, mass)
end

function World:addBeam(id, nodeA, nodeB, stiffness, damping, strength)
	local a = self.nodes[nodeA + 1]
	local b = self.nodes[nodeB + 1]
	local length = 1
	if a and b then
		length = dist3({a.x, a.y, a.z}, {b.x, b.y, b.z})
		if length < 1e-4 then length = 1e-4 end
	end
	if stiffness < 1 then stiffness = 1 end
	if damping < 0 then damping = 0 end
	if strength < 0 then strength = 0 end
	table.insert(self.beams, {
		id = id, nodeA = nodeA, nodeB = nodeB,
		stiffness = stiffness, damping = damping, strength = strength,
		yieldStrength = strength * 0.55, plasticity = 0.25,
		length = length, initialLength = length, broken = false,
		lambda = 0, stress = 0,
	})
end

function World:addTriangle(a, b, c)
	local na = self.nodes[a+1]; local nb = self.nodes[b+1]; local nc = self.nodes[c+1]
	if not (na and nb and nc) then return end
	local area = triangleArea(na.x,na.y,na.z, nb.x,nb.y,nb.z, nc.x,nc.y,nc.z)
	if area > 1e-8 then
		table.insert(self.triangles, { a=a, b=b, c=c, restArea=area, lambda=0 })
	end
end

function World:addCollisionBox(cx, cy, cz, hx, hy, hz, restitution, friction)
	table.insert(self.collisionBoxes, {
		center = {finite_or_zero(cx), finite_or_zero(cy), finite_or_zero(cz)},
		halfExtents = {math.max(math.abs(hx),1e-4), math.max(math.abs(hy),1e-4), math.max(math.abs(hz),1e-4)},
		restitution = clamp(restitution, 0, 1), friction = clamp(friction, 0, 1),
	})
end

function World:addCollisionSphere(cx, cy, cz, radius, restitution, friction)
	table.insert(self.collisionSpheres, {
		center = {finite_or_zero(cx), finite_or_zero(cy), finite_or_zero(cz)},
		radius = math.max(math.abs(radius), 1e-4),
		restitution = clamp(restitution, 0, 1), friction = clamp(friction, 0, 1),
	})
end

function World:addBoundary(x, y, z, _nx, _ny, _nz, restitution)
	table.insert(self.boundaries, {
		point = {finite_or_zero(x), finite_or_zero(y), finite_or_zero(z)},
		restitution = clamp(restitution, 0, 1), friction = 0.8,
	})
end

function World:terrainHeight(_x, _z) return 0 end
function World:terrainNormal(_x, _z) return {0, 1, 0} end
function World:rutDepthAt(_x, _z) return 0 end

function World:_updateMassProperties()
	local nodes = self.nodes
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			local baseMass = self.baseNodeMasses[i] or n.mass
			if baseMass < 1e-6 then baseMass = 1e-6 end
			n.mass = baseMass
			n.invMass = 1 / n.mass
		end
	end
end

function World:_applyForces()
	local nodes = self.nodes
	local totalMass = 0
	for i = 1, #nodes do
		if not nodes[i].fixed then totalMass += math.max(nodes[i].mass, 0) end
	end
	if totalMass <= 0 then return end
	local avgVx, avgVy, avgVz = 0, 0, 0
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			local m = math.max(n.mass, 0)
			avgVx += n.vx * m / totalMass
			avgVy += n.vy * m / totalMass
			avgVz += n.vz * m / totalMass
		end
	end
	local speed = math.sqrt(avgVx*avgVx + avgVy*avgVy + avgVz*avgVz)
	local rho = math.max(self.airDensity, 0)
	local area = math.max(self.frontalArea, 0)
	local drag = math.abs(self.dragCoefficient)
	local downforceCoeff = clamp(drag * 0.25, 0, 1)
	local downforce = math.max(0.5 * rho * area * downforceCoeff * speed * speed, 0)
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			n.fy += n.mass * self.gravity
			n.fy -= downforce * math.max(n.mass, 0) / totalMass
		end
	end
end

function World:_integrateVelocities(dt)
	local nodes = self.nodes
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			n.vx += n.fx * n.invMass * dt
			n.vy += n.fy * n.invMass * dt
			n.vz += n.fz * n.invMass * dt
			n.lastForce = math.sqrt(n.fx*n.fx + n.fy*n.fy + n.fz*n.fz)
			n.fx = 0; n.fy = 0; n.fz = 0
			n.vx = finite_or_zero(clamp(n.vx, -200, 200))
			n.vy = finite_or_zero(clamp(n.vy, -200, 200))
			n.vz = finite_or_zero(clamp(n.vz, -200, 200))
			n.x = finite_or_zero(n.x + n.vx * dt)
			n.y = finite_or_zero(n.y + n.vy * dt)
			n.z = finite_or_zero(n.z + n.vz * dt)
		end
	end
end

function World:_solveXPBD()
	local ITER = 30
	local dt = self.fixedDt
	local dt2 = dt * dt
	local nodes = self.nodes
	local beams = self.beams
	local triangles = self.triangles
	for i = 1, #beams do beams[i].lambda = 0 end
	for i = 1, #triangles do triangles[i].lambda = 0 end

	for _ = 1, ITER do
		for bi = 1, #beams do
			local beam = beams[bi]
			if not beam.broken then
				local a = beam.nodeA + 1; local b = beam.nodeB + 1
				if a <= #nodes and b <= #nodes then
					local na = nodes[a]; local nb = nodes[b]
					local dx = nb.x-na.x; local dy = nb.y-na.y; local dz = nb.z-na.z
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

		for ti = 1, #triangles do
			local tri = triangles[ti]
			local ai, bi2, ci = tri.a+1, tri.b+1, tri.c+1
			if ai <= #nodes and bi2 <= #nodes and ci <= #nodes then
				local a, b, c = nodes[ai], nodes[bi2], nodes[ci]
				local abx,aby,abz = b.x-a.x, b.y-a.y, b.z-a.z
				local acx,acy,acz = c.x-a.x, c.y-a.y, c.z-a.z
				local cr = cross3({abx,aby,abz},{acx,acy,acz})
				local doubleArea = math.sqrt(cr[1]*cr[1]+cr[2]*cr[2]+cr[3]*cr[3])
				if doubleArea >= 1e-10 then
					local nn = {cr[1]/doubleArea, cr[2]/doubleArea, cr[3]/doubleArea}
					local gA = scale3(cross3({b.x-c.x,b.y-c.y,b.z-c.z}, nn), 0.5)
					local gB = scale3(cross3({c.x-a.x,c.y-a.y,c.z-a.z}, nn), 0.5)
					local gC = scale3(cross3(nn, {abx,aby,abz}), 0.5)
					local area = doubleArea * 0.5
					local err = area - tri.restArea
					local comp = 1e-5 / math.max(dt2, 1e-12)
					local wSum = a.invMass*(gA[1]^2+gA[2]^2+gA[3]^2)
						+ b.invMass*(gB[1]^2+gB[2]^2+gB[3]^2)
						+ c.invMass*(gC[1]^2+gC[2]^2+gC[3]^2)
					local denom = wSum + comp
					if denom > 1e-12 then
						local dL = (-err - comp * tri.lambda) / denom
						tri.lambda += dL
						if a.invMass > 0 then a.x+=dL*a.invMass*gA[1]; a.y+=dL*a.invMass*gA[2]; a.z+=dL*a.invMass*gA[3] end
						if b.invMass > 0 then b.x+=dL*b.invMass*gB[1]; b.y+=dL*b.invMass*gB[2]; b.z+=dL*b.invMass*gB[3] end
						if c.invMass > 0 then c.x+=dL*c.invMass*gC[1]; c.y+=dL*c.invMass*gC[2]; c.z+=dL*c.invMass*gC[3] end
					end
				end
			end
		end
	end

	-- Beam damping
	for bi = 1, #beams do
		local beam = beams[bi]
		if not beam.broken then
			local ai = beam.nodeA+1; local bi2 = beam.nodeB+1
			if ai <= #nodes and bi2 <= #nodes then
				local a, b = nodes[ai], nodes[bi2]
				local dx,dy,dz = b.x-a.x, b.y-a.y, b.z-a.z
				local len = math.sqrt(dx*dx+dy*dy+dz*dz)
				if len > 1e-8 then
					local nx,ny,nz = dx/len,dy/len,dz/len
					local relV = (b.vx-a.vx)*nx + (b.vy-a.vy)*ny + (b.vz-a.vz)*nz
					local damp = math.max(beam.damping, 0)
					if damp > 0 then
						local effMass = a.invMass + b.invMass
						local impulse = damp * relV * dt / (1 + damp * dt * effMass)
						if not a.fixed then a.vx+=impulse*a.invMass*nx; a.vy+=impulse*a.invMass*ny; a.vz+=impulse*a.invMass*nz end
						if not b.fixed then b.vx-=impulse*b.invMass*nx; b.vy-=impulse*b.invMass*ny; b.vz-=impulse*b.invMass*nz end
					end
				end
			end
		end
	end

	-- Beam material state
	for bi = 1, #beams do
		local beam = beams[bi]
		local a = nodes[beam.nodeA+1]; local b = nodes[beam.nodeB+1]
		if a and b then
			local dx,dy,dz = b.x-a.x, b.y-a.y, b.z-a.z
			local curLen = math.sqrt(dx*dx+dy*dy+dz*dz)
			if curLen > 1e-8 then
				local axialLoad = math.abs(beam.lambda / math.max(dt2, 1e-12))
				beam.stress = axialLoad
				local tensile = beam.lambda < 0 and axialLoad or 0
				if tensile > beam.strength then
					beam.broken = true
				elseif tensile > beam.yieldStrength and beam.plasticity > 0 then
					local excess = (tensile - beam.yieldStrength) / math.max(beam.stiffness, 1)
					local inc = clamp(excess * beam.plasticity, 0, beam.initialLength * 0.02)
					beam.length = clamp(beam.length + inc, 1e-4, beam.initialLength * 1.4)
				end
			end
		end
	end

	-- Feed constraint correction
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

function World:_collideWithTerrain()
	local nodes = self.nodes
	self.telemetry[T.IMPACT_SEVERITY + 1] *= 0.90 ^ (self.fixedDt * 60)
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed and n.collision then
			local th = self:terrainHeight(n.x, n.z)
			if n.y < th then
				local normal = self:terrainNormal(n.x, n.z)
				local nv = n.vx*normal[1] + n.vy*normal[2] + n.vz*normal[3]
				if nv < 0 then
					local restitution = 0.15
					n.vx -= nv * (1 + restitution) * normal[1]
					n.vy -= nv * (1 + restitution) * normal[2]
					n.vz -= nv * (1 + restitution) * normal[3]
				end
				n.y = th
			end
		end
	end
end

function World:_collideWithStaticGeometry()
	local nodes = self.nodes
	for i = 1, #nodes do
		local n = nodes[i]
		if n.fixed or (not n.collision) then continue end
		for _, box in self.collisionBoxes do
			local rx,ry,rz = n.x-box.center[1], n.y-box.center[2], n.z-box.center[3]
			local px,py,pz = box.halfExtents[1]-math.abs(rx), box.halfExtents[2]-math.abs(ry), box.halfExtents[3]-math.abs(rz)
			if px > 0 and py > 0 and pz > 0 then
				local axis = 1; local minPen = px
				if py < minPen then axis=2; minPen=py end
				if pz < minPen then axis=3; minPen=pz end
				local normal = {0,0,0}
				local relAxis = ({rx,ry,rz})[axis]
				if math.abs(relAxis) > 1e-8 then
					normal[axis] = sign(relAxis)
				else
					normal[axis] = 1
				end
				local corr = math.min(minPen, 0.01)
				n.x += normal[1]*corr; n.y += normal[2]*corr; n.z += normal[3]*corr
				local nv = n.vx*normal[1]+n.vy*normal[2]+n.vz*normal[3]
				if nv < 0 then
					n.vx -= nv*(1+box.restitution)*normal[1]
					n.vy -= nv*(1+box.restitution)*normal[2]
					n.vz -= nv*(1+box.restitution)*normal[3]
				end
			end
		end
		for _, sphere in self.collisionSpheres do
			local rx,ry,rz = n.x-sphere.center[1], n.y-sphere.center[2], n.z-sphere.center[3]
			local dist = math.sqrt(rx*rx+ry*ry+rz*rz)
			if dist < sphere.radius then
				local normal
				if dist > 1e-8 then normal = {rx/dist,ry/dist,rz/dist} else normal = {0,1,0} end
				local corr = math.min(sphere.radius-dist, 0.01)
				n.x+=normal[1]*corr; n.y+=normal[2]*corr; n.z+=normal[3]*corr
				local nv = n.vx*normal[1]+n.vy*normal[2]+n.vz*normal[3]
				if nv < 0 then
					n.vx-=(1+sphere.restitution)*nv*normal[1]
					n.vy-=(1+sphere.restitution)*nv*normal[2]
					n.vz-=(1+sphere.restitution)*nv*normal[3]
				end
			end
		end
	end
end

function World:_applyDrag()
	local nodes = self.nodes
	local totalMass = 0
	for i = 1, #nodes do
		if not nodes[i].fixed then totalMass += nodes[i].mass end
	end
	totalMass = math.max(totalMass, 1)
	local avgVx, avgVy, avgVz = 0, 0, 0
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			avgVx += n.vx*n.mass/totalMass
			avgVy += n.vy*n.mass/totalMass
			avgVz += n.vz*n.mass/totalMass
		end
	end
	local speed = math.sqrt(avgVx*avgVx+avgVy*avgVy+avgVz*avgVz)
	local totalDrag = 0.5*self.airDensity*self.dragCoefficient*self.frontalArea*speed*speed
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			local ns = math.sqrt(n.vx*n.vx+n.vy*n.vy+n.vz*n.vz)
			if ns > 1e-6 then
				local drag = totalDrag*n.mass/totalMass
				local factor = math.max(1-drag*self.fixedDt/(n.mass*ns+1e-6),0)
				n.vx*=factor; n.vy*=factor; n.vz*=factor
			end
		end
	end
end

function World:_updateTelemetry()
	local t = self.telemetry
	local nodes = self.nodes
	local avgVx, avgVz = 0, 0; local dynCount = 0
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then avgVx += n.vx; avgVz += n.vz; dynCount += 1 end
	end
	if dynCount > 0 then avgVx /= dynCount; avgVz /= dynCount end
	local speed = math.sqrt(avgVx*avgVx + avgVz*avgVz)
	t[T.SPEED_MPS + 1] = speed
	t[T.SPEED_KMH + 1] = speed * 3.6
	t[T.SIM_TIME + 1] = self.time
	local broken = 0
	for _, beam in self.beams do if beam.broken then broken += 1 end end
	t[T.BROKEN_BEAMS + 1] = broken
end

function World:_updateVehicleSystems() end

function World:stepFixed()
	local count = #self.nodes
	if count == 0 then return end
	while #self.constraintStartPositions < count do
		table.insert(self.constraintStartPositions, {0,0,0})
	end
	self:_updateVehicleSystems()
	self:_applyForces()
	self:_integrateVelocities(self.fixedDt)
	for i = 1, count do
		local n = self.nodes[i]; local sp = self.constraintStartPositions[i]
		sp[1] = n.x; sp[2] = n.y; sp[3] = n.z
	end
	self:_solveXPBD()
	self:_collideWithTerrain()
	self:_collideWithStaticGeometry()
	self:_applyDrag()
	self:_updateTelemetry()
end

function World:step(elapsed)
	self.accumulator += elapsed
	local steps = 0
	while self.accumulator >= self.fixedDt and steps < self.maxSubsteps do
		self:stepFixed()
		self.time += self.fixedDt
		self.accumulator -= self.fixedDt
		steps += 1
	end
	if self.accumulator > self.fixedDt * self.maxSubsteps then self.accumulator = 0 end
end

-- ============================================================
-- Resolve a dotted path like "nodes[0].positionM[1]"
-- ============================================================

-- Map fixture path names to World internal field names
local FIELD_MAP = {
	positionM = { "x", "y", "z" },
	velocityMps = { "vx", "vy", "vz" },
}

local function resolve(path, state)
	local val = state
	for token in path:gmatch("[^%.]+") do
		local key, idx = token:match("^(%w+)%[(%d+)%]")
		if key and idx then
			idx = tonumber(idx)
			if key == "nodes" or key == "beams" then
				val = val[key]
				if val == nil then return nil end
				val = val[idx + 1] -- 0-indexed → 1-indexed
			elseif key == "telemetry" then
				val = val[key]
				if val == nil then return nil end
				val = val[idx + 1] -- 0-indexed → 1-indexed
			else
				-- Check if this is a mapped field (e.g. positionM[1] → node.y)
				local mapping = FIELD_MAP[key]
				if mapping then
					val = val[mapping[idx + 1]]
				else
					val = val[key]
					if val == nil then return nil end
					val = val[idx]
				end
			end
		else
			val = val[token]
		end
		if val == nil then return nil end
	end
	return val
end

local function assert_check(actual, op, expected, tol, desc)
	if actual == nil then return false, "path resolved to nil" end
	if op == "equals" then
		if actual ~= expected then return false, string.format("expected %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "approximatelyEquals" then
		if math.abs(actual - expected) > (tol or 1e-6) then
			return false, string.format("expected ~%.10f, got %.10f (tol %.10f)", expected, actual, tol or 1e-6)
		end
	elseif op == "lessThan" then
		if not (actual < expected) then return false, string.format("expected < %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "greaterThan" then
		if not (actual > expected) then return false, string.format("expected > %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "lessThanOrEquals" then
		if not (actual <= expected) then return false, string.format("expected <= %s, got %s", tostring(expected), tostring(actual)) end
	elseif op == "greaterThanOrEquals" then
		if not (actual >= expected) then return false, string.format("expected >= %s, got %s", tostring(expected), tostring(actual)) end
	else
		return false, "unknown operator: " .. tostring(op)
	end
	return true, ""
end

-- ============================================================
-- Load fixture, simulate, check assertions
-- ============================================================

local function runFixture(path)
	local raw = fs.readFile(path)
	local fixture = serde.decode("json", raw)

	local world = World.new()
	if fixture.initial then
		world.gravity = fixture.initial.gravityMps2 or GRAVITY
		world.fixedDt = fixture.initial.fixedDtSeconds or (1/60)

		for _, n in fixture.initial.nodes do
			world:addNode(n.id, n.positionM[1], n.positionM[2], n.positionM[3],
				n.massKg or 1, n.fixed or false, n.collision)
		end
		for _, b in (fixture.initial.beams or {}) do
			world:addBeam(b.id, b.nodeA, b.nodeB,
				b.stiffness or 10000, b.damping or 100, b.strength or 5000)
		end
		for _, tri in (fixture.initial.triangles or {}) do
			world:addTriangle(tri.a, tri.b, tri.c)
		end
	end

	-- Replay steps
	for _, step in (fixture.steps or {}) do
		local dt = step.elapsedSeconds or world.fixedDt
		if step.controls then
			for k, v in pairs(step.controls) do
				world.controls[k] = v
			end
		end
		world:step(dt)
	end

	-- Check assertions
	local pass, fail = 0, 0
	for _, a in (fixture.assertions or {}) do
		local val = resolve(a.path, world)
		local ok, msg = assert_check(val, a.operator, a.value, a.absoluteTolerance, a.description)
		if ok then
			pass += 1
		else
			fail += 1
			print(string.format("    FAIL: %s — %s (got %s)", a.path, msg, tostring(val)))
		end
	end

	return pass, fail
end

-- ============================================================
-- Main
-- ============================================================

local FIXTURES_DIR = "/Users/darelisme/Documents/Development/kemudi.js/fixtures/engine"

-- Get fixture paths from args or discover all
local fixturePaths = {}
local args = process.args
if #args > 2 then
	-- lune run test/fixture_runner.luau <fixture_name> [fixture_name...]
	for i = 3, #args do
		table.insert(fixturePaths, FIXTURES_DIR .. "/" .. args[i] .. ".json")
	end
else
	-- Discover all .json fixtures
	local entries = fs.readDir(FIXTURES_DIR)
	for _, entry in entries do
		if entry:match("%.json$") then
			table.insert(fixturePaths, FIXTURES_DIR .. "/" .. entry)
		end
	end
	table.sort(fixturePaths)
end

print("=== Kemudi Engine Fixture Runner ===\n")

local totalPass, totalFail = 0, 0
local fixturePass, fixtureFail = 0, 0

for _, fpath in fixturePaths do
	local name = fpath:match("([^/]+)%.json$")
	io = io or {} -- luau: Lune has no io global
	local ok, p, f = pcall(function()
		return runFixture(fpath)
	end)
	if ok then
		totalPass += p; totalFail += f
		if f == 0 then
			print(string.format("PASS (%d/%d)", p, p))
			fixturePass += 1
		else
			print(string.format("FAIL (%d/%d)", p, p + f))
			fixtureFail += 1
		end
	else
		print("ERROR: " .. tostring(p))
		fixtureFail += 1
		totalFail += 1
	end
end

print(string.format("\n=== Results: %d/%d fixtures passed, %d/%d assertions passed ===",
	fixturePass, fixturePass + fixtureFail, totalPass, totalPass + totalFail))
if totalFail > 0 or fixtureFail > 0 then
	process.exit(1)
end
