--[[
	Physics world — the core simulation container.
	Matches Rust PhysicsWorld from crates/kemudi-engine/src/types.rs and world.rs.
	All data is stored in flat tables for cache-friendly access patterns.
--]]

local Contract = require(script.Parent.contract)
local Math = require(script.Parent.math)

-- Vehicle subsystems (loaded lazily to avoid circular deps)
local Suspension = require(script.Parent.Parent.vehicle.suspension)
local Tires = require(script.Parent.Parent.vehicle.tires)
local Drivetrain = require(script.Parent.Parent.vehicle.drivetrain)
local Safety = require(script.Parent.Parent.vehicle.safety)

local World = {}
World.__index = World

local function makeTelemetry()
	local t = table.create(Contract.TELEMETRY_LEN, 0)
	t[Contract.T.DERATE + 1] = 1.0
	return t
end

function World.new()
	local self = setmetatable({}, World)
	self.nodes = {}
	self.beams = {}
	self.triangles = {}
	self.gravity = Contract.DEFAULT_GRAVITY
	self.time = 0
	self.airDensity = 1.225
	self.dragCoefficient = 0.5
	self.frontalArea = 2.2
	self.accumulator = 0
	self.fixedDt = Contract.DEFAULT_FIXED_DT
	self.maxSubsteps = Contract.DEFAULT_MAX_SUBSTEPS
	self.restPositions = {}
	self.baseNodeMasses = {}
	self.constraintStartPositions = {}
	self.controls = {
		steering = 0, throttle = 0, brake = 0, clutch = 0,
		handbrake = false, gearUp = false, gearDown = false, engineOn = false,
	}
	self.terrainProfile = 0
	local gridSize = Contract.TERRAIN_GRID_SIZE
	self.terrainRuts = table.create(gridSize * gridSize, 0)
	self.terrainHeights = {}
	self.terrainWidth = 0
	self.terrainDepth = 0
	self.terrainSegments = 0
	self.groundFriction = 0.94
	self.surfaceRoughness = 0.65
	self.surfaceMoisture = 0
	self.surfaceCompactness = 1
	self.surfacePresetIndex = 0
	self.collisionBoxes = {}
	self.collisionSpheres = {}
	self.boundaries = {}
	self.roadSurfaces = {}
	self.telemetry = makeTelemetry()

	-- Vehicle systems (optional — set up via World:setupVehicle)
	self.suspension_config = nil
	self.suspension_states = nil
	self.steering_config = nil
	self.steering_angle = 0.0
	self.tires = nil       -- array of 4 TireState
	self.tire_thermal = nil
	self.drivetrain = nil
	self.safety = nil
	self.fuel_tank = nil
	self.vehicle_mass = 0

	return self
end

-- Set up a full vehicle with 4-wheel suspension, tires, drivetrain, safety.
function World:setupVehicle(suspension_cfg, steering_cfg, tire_compounds, vehicle_mass)
	self.suspension_config = suspension_cfg or Suspension.newConfig()
	self.steering_config = steering_cfg or Suspension.newSteeringConfig()
	self.vehicle_mass = vehicle_mass or 1500

	-- 4 wheel states
	self.suspension_states = {}
	for _ = 1, 4 do
		table.insert(self.suspension_states, Suspension.newWheelState())
	end

	-- 4 tires
	self.tires = {}
	for i = 1, 4 do
		table.insert(self.tires, Tires.newState(tire_compounds and tire_compounds[i] or "Street"))
	end
	self.tire_thermal = Tires.newThermalParams()

	-- Drivetrain
	self.drivetrain = Drivetrain.new()
	self.drivetrain.wheel_radius = self.suspension_config.wheels[1].tire_radius

	-- Safety
	self.safety = Safety.newState()

	-- Fuel
	self.fuel_tank = Suspension.newFuelTank()
end

-- Add a node, matching Rust PhysicsWorld::add_node.
function World:addNode(id, x, y, z, mass, fixed)
	x = Math.finite_or_zero(x)
	y = Math.finite_or_zero(y)
	z = Math.finite_or_zero(z)
	if not (mass > 0 and mass == mass) then mass = 1 end -- NaN/guard
	local invMass = if fixed then 0 else 1 / mass
	local nodeCount = #self.nodes
	table.insert(self.nodes, {
		id = id,
		x = x, y = y, z = z,
		vx = 0, vy = 0, vz = 0,
		mass = mass,
		invMass = invMass,
		fixed = fixed,
		-- First four nodes are suspension mounts, not collision colliders.
		-- Upper-cage nodes (index >= 4) get collision enabled.
		collision = (not fixed) and nodeCount >= 4,
		fx = 0, fy = 0, fz = 0,
		lastForce = 0,
	})
	table.insert(self.constraintStartPositions, {x, y, z})
	table.insert(self.restPositions, {x, y, z})
	table.insert(self.baseNodeMasses, mass)
end

function World:setNodeCollision(nodeId, collision)
	local node = self.nodes[nodeId + 1] -- 0-indexed externally
	if node then
		node.collision = (nodeId >= 4) and (not node.fixed) and collision
	end
end

-- Add a beam between two node indices (0-based).
function World:addBeam(id, nodeA, nodeB, stiffness, damping, strength)
	local a = self.nodes[nodeA + 1]
	local b = self.nodes[nodeB + 1]
	local length = 1
	if a and b then
		length = Math.distance3(
			{a.x, a.y, a.z}, {b.x, b.y, b.z}
		)
		if length < 1e-4 then length = 1e-4 end
	end
	if stiffness < 1 then stiffness = 1 end
	if damping < 0 then damping = 0 end
	if strength < 0 then strength = 0 end
	table.insert(self.beams, {
		id = id,
		nodeA = nodeA,
		nodeB = nodeB,
		stiffness = stiffness,
		damping = damping,
		strength = strength,
		yieldStrength = strength * 0.55,
		plasticity = 0.25,
		length = length,
		initialLength = length,
		broken = false,
		lambda = 0,
		stress = 0,
	})
end

function World:setBeamMaterial(beamId, yieldStrength, plasticity)
	for _, beam in self.beams do
		if beam.id == beamId then
			if yieldStrength == yieldStrength then -- finite check
				beam.yieldStrength = math.clamp(yieldStrength, 0, beam.strength)
			end
			if plasticity == plasticity then
				beam.plasticity = math.clamp(plasticity, 0, 1)
			end
			break
		end
	end
end

-- Add a triangle area-preservation constraint.
function World:addTriangle(a, b, c)
	local na = self.nodes[a + 1]
	local nb = self.nodes[b + 1]
	local nc = self.nodes[c + 1]
	if not (na and nb and nc) then return end
	local area = Math.triangleArea(
		na.x, na.y, na.z,
		nb.x, nb.y, nb.z,
		nc.x, nc.y, nc.z
	)
	if area > 1e-8 then
		table.insert(self.triangles, { a = a, b = b, c = c, restArea = area, lambda = 0 })
	end
end

function World:addCollisionBox(cx, cy, cz, hx, hy, hz, restitution, friction)
	table.insert(self.collisionBoxes, {
		center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
		halfExtents = {
			math.max(math.abs(hx), 1e-4),
			math.max(math.abs(hy), 1e-4),
			math.max(math.abs(hz), 1e-4),
		},
		restitution = math.clamp(restitution, 0, 1),
		friction = math.clamp(friction, 0, 1),
	})
end

function World:addCollisionSphere(cx, cy, cz, radius, restitution, friction)
	table.insert(self.collisionSpheres, {
		center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
		radius = math.max(math.abs(radius), 1e-4),
		restitution = math.clamp(restitution, 0, 1),
		friction = math.clamp(friction, 0, 1),
	})
end

function World:addBoundary(x, y, z, _nx, _ny, _nz, restitution)
	table.insert(self.boundaries, {
		point = { Math.finite_or_zero(x), Math.finite_or_zero(y), Math.finite_or_zero(z) },
		restitution = math.clamp(restitution, 0, 1),
		friction = 0.8,
	})
end

function World:addRoadSurface(points, width, surfaceId, friction, roughness, moisture, compactness)
	local polyline = {}
	for i = 1, #points, 2 do
		local px, pz = points[i], points[i + 1]
		if px == px and pz == pz then -- finite check
			table.insert(polyline, { px, pz })
		end
	end
	if #polyline < 2 then return end
	table.insert(self.roadSurfaces, {
		points = polyline,
		width = math.clamp(math.abs(width), 0.01, 10000),
		surfaceId = surfaceId,
		friction = math.clamp(friction, 0, 2),
		roughness = math.clamp(roughness, 0, 1),
		moisture = math.clamp(moisture, 0, 1),
		compactness = math.clamp(compactness, 0, 1),
	})
end

function World:setDrivetrainLayout(layout)
	self.driveLayout = math.min(layout, 2)
end

-- Terrain height query (profile fallback or heightmap).
function World:terrainHeight(x, z)
	local baseHeight = 0
	if self.terrainSegments >= 1 and #self.terrainHeights > 0 then
		local seg = self.terrainSegments
		local gx = math.clamp((x / self.terrainWidth + 0.5) * seg, 0, seg)
		local gz = math.clamp((z / self.terrainDepth + 0.5) * seg, 0, seg)
		local ix = math.floor(gx)
		local iz = math.floor(gz)
		local x1 = math.min(ix + 1, seg)
		local z1 = math.min(iz + 1, seg)
		local fx = gx - ix
		local fz = gz - iz
		local stride = seg + 1
		local h00 = self.terrainHeights[iz * stride + ix + 1]
		local h10 = self.terrainHeights[iz * stride + x1 + 1]
		local h01 = self.terrainHeights[z1 * stride + ix + 1]
		local h11 = self.terrainHeights[z1 * stride + x1 + 1]
		baseHeight = (h00 + (h10 - h00) * fx) + ((h01 + (h11 - h01) * fx) - (h00 + (h10 - h00) * fx)) * fz
	else
		baseHeight = Math.terrainHeightForProfile(self.terrainProfile, x, z)
	end
	return baseHeight - self:rutDepthAt(x, z)
end

function World:rutDepthAt(x, z)
	local gridSize = Contract.TERRAIN_GRID_SIZE
	local extent = Contract.TERRAIN_EXTENT
	local gx = math.clamp(math.floor((x / extent + 0.5) * gridSize), 0, gridSize - 1)
	local gz = math.clamp(math.floor((z / extent + 0.5) * gridSize), 0, gridSize - 1)
	return self.terrainRuts[gz * gridSize + gx + 1]
end

function World:terrainNormal(x, z)
	local e = 0.25
	local hL = self:terrainHeight(x - e, z)
	local hR = self:terrainHeight(x + e, z)
	local hD = self:terrainHeight(x, z - e)
	local hU = self:terrainHeight(x, z + e)
	local nx, ny, nz = hL - hR, 2 * e, hD - hU
	local len = math.sqrt(nx * nx + ny * ny + nz * nz)
	if len < 1e-6 then len = 1e-6 end
	return { nx / len, ny / len, nz / len }
end

--[[
	Internal simulation methods.
	Split into methods on World to keep the hot path in one file.
--]]

function World:_updateMassProperties()
	local nodes = self.nodes
	local count = #nodes
	local dynamicCount = 0
	for i = 1, count do
		if not nodes[i].fixed then dynamicCount += 1 end
	end
	if dynamicCount == 0 then return end
	-- Fuel mass distribution (simplified for now)
	local fuelShare = 0 -- TODO: fuel system
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local baseMass = self.baseNodeMasses[i] or n.mass
			if baseMass < 1e-6 then baseMass = 1e-6 end
			n.mass = baseMass + fuelShare
			n.invMass = 1 / n.mass
		end
	end
end

function World:_applyForces()
	local nodes = self.nodes
	local count = #nodes
	-- Compute total mass
	local totalMass = 0
	for i = 1, count do
		if not nodes[i].fixed then
			totalMass += math.max(nodes[i].mass, 0)
		end
	end
	if totalMass <= 0 then return end

	-- Average velocity
	local avgVx, avgVy, avgVz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local m = math.max(n.mass, 0)
			avgVx += n.vx * m / totalMass
			avgVy += n.vy * m / totalMass
			avgVz += n.vz * m / totalMass
		end
	end
	local speed = math.sqrt(avgVx * avgVx + avgVy * avgVy + avgVz * avgVz)
	-- Downforce (simplified — same formula as Rust body_downforce_force)
	local safeSpeed = math.abs(speed)
	local rho = math.max(self.airDensity, 0)
	local area = math.max(self.frontalArea, 0)
	local drag = math.abs(self.dragCoefficient)
	local downforceCoeff = math.clamp(drag * 0.25, 0, 1)
	local downforce = math.max(0.5 * rho * area * downforceCoeff * safeSpeed * safeSpeed, 0)

	-- Apply gravity and downforce
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			n.fy += n.mass * self.gravity
			n.fy -= downforce * math.max(n.mass, 0) / totalMass
		end
	end
end

function World:_integrateVelocities(dt: number)
	local nodes = self.nodes
	local count = #nodes
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			n.vx += n.fx * n.invMass * dt
			n.vy += n.fy * n.invMass * dt
			n.vz += n.fz * n.invMass * dt
			n.lastForce = math.sqrt(n.fx * n.fx + n.fy * n.fy + n.fz * n.fz)
			n.fx = 0
			n.fy = 0
			n.fz = 0
			n.vx = Math.finite_or_zero(math.clamp(n.vx, -200, 200))
			n.vy = Math.finite_or_zero(math.clamp(n.vy, -200, 200))
			n.vz = Math.finite_or_zero(math.clamp(n.vz, -200, 200))
			n.x = Math.finite_or_zero(n.x + n.vx * dt)
			n.y = Math.finite_or_zero(n.y + n.vy * dt)
			n.z = Math.finite_or_zero(n.z + n.vz * dt)
		end
	end
end

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
				local crx, cry, crz = Math.cross3({abx,aby,abz},{acx,acy,acz})
				local doubleArea = math.sqrt(crx*crx + cry*cry + crz*crz)
				if doubleArea >= 1e-10 then
					local nn = { crx/doubleArea, cry/doubleArea, crz/doubleArea }
					-- Gradients of 0.5 * |ab x ac|
					local gradAx, gradAy, gradAz = Math.scale3(Math.cross3({b.x-c.x,b.y-c.y,b.z-c.z}, nn), 0.5)
					local gradBx, gradBy, gradBz = Math.scale3(Math.cross3({c.x-a.x,c.y-a.y,c.z-a.z}, nn), 0.5)
					local gradCx, gradCy, gradCz = Math.scale3(Math.cross3(nn, {abx,aby,abz}), 0.5)
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
	end

	-- Beam damping (once after iterations)
	self:_applyBeamDamping(dt)
	self:_updateBeamMaterialState(dt)

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
				local axialLoad = math.abs(beam.lambda / math.max(dt2, 1e-12))
				beam.stress = axialLoad
				local tensile = beam.lambda < 0 and axialLoad or 0
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
				if n.y < th then
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
			if n.y < boundary.point[2] then
				local corr = math.min(boundary.point[2] - n.y, 0.01)
				n.y += corr
				self:_applyContactVelocity(n, {0, 1, 0}, boundary.restitution, boundary.friction)
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

-- Aerodynamic drag
function World:_applyDrag()
	local nodes = self.nodes
	local count = #nodes
	local totalMass = 0
	for i = 1, count do
		if not nodes[i].fixed then totalMass += nodes[i].mass end
	end
	totalMass = math.max(totalMass, 1)
	local avgVx, avgVy, avgVz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			avgVx += n.vx * n.mass / totalMass
			avgVy += n.vy * n.mass / totalMass
			avgVz += n.vz * n.mass / totalMass
		end
	end
	local speed = math.sqrt(avgVx*avgVx + avgVy*avgVy + avgVz*avgVz)
	local totalDrag = 0.5 * self.airDensity * self.dragCoefficient * self.frontalArea * speed * speed
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local ns = math.sqrt(n.vx*n.vx + n.vy*n.vy + n.vz*n.vz)
			if ns > 1e-6 then
				local drag = totalDrag * n.mass / totalMass
				local factor = math.max(1 - drag * self.fixedDt / (n.mass * ns + 1e-6), 0)
				n.vx *= factor
				n.vy *= factor
				n.vz *= factor
			end
		end
	end
end

-- Telemetry update
function World:_updateTelemetry()
	local t = self.telemetry
	local nodes = self.nodes
	local count = #nodes
	-- Speed from average vehicle velocity
	local avgVx, avgVz = 0, 0
	local dynCount = 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			avgVx += n.vx
			avgVz += n.vz
			dynCount += 1
		end
	end
	if dynCount > 0 then
		avgVx /= dynCount
		avgVz /= dynCount
	end
	local speed = math.sqrt(avgVx * avgVx + avgVz * avgVz)
	t[Contract.T.SPEED_MPS + 1] = speed
	t[Contract.T.SPEED_KMH + 1] = speed * 3.6
	-- Sim time
	t[Contract.T.SIM_TIME + 1] = self.time
	-- Broken beams count
	local broken = 0
	for _, beam in self.beams do
		if beam.broken then broken += 1 end
	end
	t[Contract.T.BROKEN_BEAMS + 1] = broken
end

-- Vehicle system update — called each fixed step before forces.
function World:_updateVehicleSystems()
	if not self.drivetrain then return end
	local dt = self.fixedDt
	local count = #self.nodes

	-- Update mass properties first
	self:_updateMassProperties()

	-- Fuel consumption
	if self.fuel_tank and self.drivetrain then
		Suspension.updateFuel(self.fuel_tank, self.drivetrain.engine.rpm,
			self.drivetrain.throttle_input, self.drivetrain.engine.redline_rpm, dt)
	end

	-- Steering
	if self.steering_config and count >= 2 then
		-- Vehicle speed from average node velocity
		local avg_vx, avg_vz = 0, 0
		local dyn = 0
		for i = 1, count do
			local n = self.nodes[i]
			if not n.fixed then avg_vx += n.vx; avg_vz += n.vz; dyn += 1 end
		end
		if dyn > 0 then avg_vx /= dyn; avg_vz /= dyn end
		local speed = math.sqrt(avg_vx^2 + avg_vz^2)

		local input = (self.controls and self.controls.steering or 0) * self.steering_config.max_steering_angle
		self.steering_angle = Suspension.updateSteering(
			self.steering_config, input, speed, self.steering_angle, dt)

		-- Ackermann for front wheels
		local left_angle, right_angle = Suspension.ackermannAngles(
			self.steering_angle, self.steering_config.wheelbase, self.steering_config.track_width)
		-- Store for telemetry (node 0=FL, 1=FR)
		if self.steering_states then
			self.steering_states[1] = left_angle
			self.steering_states[2] = right_angle
		end
	end

	-- Drivetrain update
	local d = self.drivetrain
	d.throttle_input = (self.controls and self.controls.throttle or 0)
	d.brake_input = (self.controls and self.controls.brake or 0)
	d.clutch_input = (self.controls and self.controls.clutch or 0)

	-- Compute vehicle speed from chassis nodes
	if count >= 1 then
		local n = self.nodes[1]
		d.wheel_speed = math.sqrt(n.vx^2 + n.vz^2) / math.max(d.wheel_radius, 0.01)
	end

	-- Manual shift requests
	if self.controls then
		if self.controls.gear_up then Drivetrain.requestShiftUp(d) end
		if self.controls.gear_down then Drivetrain.requestShiftDown(d) end
	end

	-- Auto-shift
	Drivetrain.applyAutomaticShifting(d)
	Drivetrain.update(dt, self.vehicle_mass, d)

	-- Apply wheel torque to driven wheels (nodes 0-3 = FL, FR, RL, RR)
	local drive_torque = d.last_drive_torque
	-- Differential split for rear-wheel drive (nodes 2,3)
	if count >= 4 and d.differential then
		local left_torque, right_torque = Drivetrain.applyDifferential(
			d.differential, drive_torque, 1, 1, 0, 0)
		-- Apply as forces to rear wheels (simplified)
		local torque_per_mass = left_torque / math.max(self.vehicle_mass, 1)
		local n3 = self.nodes[3] -- rear-left
		if n3 and not n3.fixed then
			n3.fz += torque_per_mass * 0.01
		end
		local n4 = self.nodes[4] -- rear-right
		if n4 and not n4.fixed then
			n4.fz += (right_torque / math.max(self.vehicle_mass, 1)) * 0.01
		end
	end

	-- Tire thermal/wear update
	if self.tires and self.tire_thermal then
		for i = 1, math.min(#self.tires, 4) do
			local n = self.nodes[i]
			if n and not n.fixed then
				local load = math.max(-n.fy, 0) -- suspension pushes up, node feels down
				Tires.update(self.tires[i], self.tire_thermal,
					0, 0, load, 0.5, 20, dt)
			end
		end
	end

	-- Safety systems (placeholder — full integration needs per-wheel data)
	-- Will be wired when suspension wheel data is fully connected
end

-- Fixed-step physics pipeline matching spec/README.md.
function World:stepFixed()
	local nodes = self.nodes
	local beams = self.beams
	local triangles = self.triangles
	local count = #nodes
	if count == 0 then return end

	-- Resize constraint start positions
	while #self.constraintStartPositions < count do
		table.insert(self.constraintStartPositions, {0, 0, 0})
	end

	-- 1. Update vehicle systems (suspension, tires, drivetrain, safety)
	self:_updateVehicleSystems()

	-- 2. Gravity and downforce
	self:_applyForces()

	-- 3. Integrate velocities
	self:_integrateVelocities(self.fixedDt)

	-- Save unconstrained positions
	for i = 1, count do
		local n = nodes[i]
		local sp = self.constraintStartPositions[i]
		sp[1] = n.x
		sp[2] = n.y
		sp[3] = n.z
	end

	-- 4. XPBD constraint solver
	self:_solveXPBD()

	-- 5. Terrain + static collision
	self:_collideWithTerrain()
	self:_collideWithStaticGeometry()

	-- 6. Aerodynamic drag
	self:_applyDrag()

	-- 7. Telemetry
	self:_updateTelemetry()
end

-- Public step: accumulate dt and run fixed substeps.
function World:step(elapsed)
	self.accumulator += elapsed
	local steps = 0
	while self.accumulator >= self.fixedDt and steps < self.maxSubsteps do
		self:stepFixed()
		self.time += self.fixedDt
		self.accumulator -= self.fixedDt
		steps += 1
	end
	-- Discard excess accumulator
	if self.accumulator > self.fixedDt * self.maxSubsteps then
		self.accumulator = 0
	end
end

return World
