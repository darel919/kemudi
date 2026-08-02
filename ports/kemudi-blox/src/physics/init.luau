--[[
	Physics world — the core simulation container.
	Matches Rust PhysicsWorld from crates/kemudi-engine/src/types.rs and world.rs.
	All data is stored in flat tables for cache-friendly access patterns.
--]]

local Contract = if script then require(script.Parent.contract) else require("../contract/init")
local Math = if script then require(script.Parent.math) else require("../math")

-- Vehicle subsystems (loaded lazily to avoid circular deps)
local Suspension = if script then require(script.Parent.vehicle.suspension) else require("../vehicle/suspension")
local Tires = if script then require(script.Parent.vehicle.tires) else require("../vehicle/tires")
local Drivetrain = if script then require(script.Parent.vehicle.drivetrain) else require("../vehicle/drivetrain")
local Safety = if script then require(script.Parent.vehicle.safety) else require("../vehicle/safety")
local EngineThermal = if script then require(script.Parent.vehicle.engine_thermal) else require("../vehicle/engine_thermal")

local Integration = if script then require(script.integration) else require("./integration")
local Constraints = if script then require(script.constraints) else require("./constraints")
local Collision = if script then require(script.collision) else require("./collision")
local Aerodynamics = if script then require(script.aerodynamics) else require("./aerodynamics")
local Telemetry = if script then require(script.telemetry) else require("./telemetry")
local Vehicle = if script then require(script.vehicle) else require("./vehicle")

local World = {}
World.__index = World

Integration.install(World, Math)
Constraints.install(World, Math)
Collision.install(World, Contract, Math)
Aerodynamics.install(World)
Telemetry.install(World, Contract, Safety)
Vehicle.install(World, Suspension, Tires, Drivetrain, Safety, Math, EngineThermal)

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
		engine_on = false, engine_stage = 0,
		selector_up = false, selector_down = false,
		manual_shift_up = false, manual_shift_down = false,
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

	-- Volume preservation state
	self.restVolume = 0
	self.restCentroid = { 0, 0, 0 }
	self.volumeLambda = 0
	self.volumeCompliance = 1e-5 -- XPBD compliance (higher = softer)
	self.bodyAttachmentSoftness = 0.35 -- 0 = rigid, 1 = no constraint

	-- Vehicle systems (optional — set up via World:setupVehicle)
	self.suspension_config = nil
	self.suspension_states = nil
	self.steering_config = nil
	self.steering_angle = 0.0
	self.steering_states = { 0.0, 0.0 }
	self.tires = nil       -- array of 4 TireState
	self.tire_thermal = nil
	self.tire_params = nil
	self.drivetrain = nil
	self.safety = nil
	self.fuel_tank = nil
	self.engine_thermal = nil
	self._previousSpeed = 0
	self.vehicle_mass = 0
	self.driveLayout = 0 -- 0 = RWD, 1 = FWD, 2 = AWD
	self.previousHeading = 0.0
	self.yawRate = 0.0

	return self
end

-- Set up a full vehicle with 4-wheel suspension, tires, drivetrain, safety.
function World:setupVehicle(suspension_cfg, steering_cfg, tire_compounds, vehicle_mass, drivetrain_config)
	assert(type(vehicle_mass) == "number" and vehicle_mass > 0, "vehicle_mass must come from validated asset mass proxies")
	self.suspension_config = suspension_cfg or Suspension.newConfig()
	self.steering_config = steering_cfg or Suspension.newSteeringConfig()
	self.vehicle_mass = vehicle_mass

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
	self.tire_params = Tires.newPacejkaParams()

	-- Drivetrain
	self.drivetrain = Drivetrain.new(drivetrain_config)
	self.drivetrain.wheel_radius = self.suspension_config.wheels[1].tire_radius
	self.drivetrain.tcm.wheel_radius = self.drivetrain.wheel_radius

	-- Safety
	self.safety = Safety.newState()

	-- Fuel
	self.fuel_tank = Suspension.newFuelTank()
	-- Engine thermal (coolant, oil, lubrication, damage, stress)
	self.engine_thermal = EngineThermal.new()
	self.previousHeading = 0.0
	self.yawRate = 0.0
end

-- Add a node, matching Rust PhysicsWorld::add_node.
function World:addNode(id, x, y, z, mass, fixed, staticCollision)
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
		-- Suspension nodes are excluded from analytic terrain collision because
		-- suspension resolves their ground contact, but every dynamic node must
		-- still be eligible for authored static map geometry.
		collision = (not fixed) and nodeCount >= 4,
		staticCollision = if staticCollision == nil then not fixed else (not fixed) and staticCollision == true,
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
		node.staticCollision = (not node.fixed) and collision
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

function World:addCollisionBox(cx, cy, cz, hx, hy, hz, restitution, friction, right, up, forward)
	local basis = nil
	if type(right) == "table" and type(up) == "table" and type(forward) == "table" then
		basis = { right, up, forward }
	end
	table.insert(self.collisionBoxes, {
		center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
		halfExtents = {
			math.max(math.abs(hx), 1e-4),
			math.max(math.abs(hy), 1e-4),
			math.max(math.abs(hz), 1e-4),
		},
		restitution = math.clamp(restitution, 0, 1),
		friction = math.clamp(friction, 0, 1),
		basis = basis,
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

function World:addBoundary(x, y, z, nx, ny, nz, restitution)
	local length = math.sqrt(nx * nx + ny * ny + nz * nz)
	if length < 1e-8 or length ~= length then
		nx, ny, nz = 0, 1, 0
	else
		nx, ny, nz = nx / length, ny / length, nz / length
	end
	table.insert(self.boundaries, {
		point = { Math.finite_or_zero(x), Math.finite_or_zero(y), Math.finite_or_zero(z) },
		normal = { nx, ny, nz },
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

-- Auto-generate triangles from the beam graph (finds all 3-cycles).
-- Call after all beams and nodes are added, before computeRestVolume.
function World:autoGenerateTriangles()
	local adj = {}
	for i = 1, #self.nodes do adj[i] = {} end
	for _, beam in self.beams do
		if not beam.broken then
			local a, b = beam.nodeA + 1, beam.nodeB + 1
			if a <= #self.nodes and b <= #self.nodes then
				adj[a][b] = true
				adj[b][a] = true
			end
		end
	end
	for i = 1, #self.nodes do
		for j in adj[i] do
			if j > i then
				for k in adj[j] do
					if k > j and adj[i][k] then
						self:addTriangle(i - 1, j - 1, k - 1)
					end
				end
			end
		end
	end
end

-- Compute the rest volume and centroid of the triangle mesh.
-- Call once after autoGenerateTriangles or manual triangle setup.
function World:computeRestVolume()
	local triangles = self.triangles
	local nodes = self.nodes
	if #triangles == 0 then return end

	-- Centroid of all dynamic nodes
	local ccx, ccy, czz, count = 0, 0, 0, 0
	for i = 1, #nodes do
		if not nodes[i].fixed then
			ccx += nodes[i].x
			ccy += nodes[i].y
			czz += nodes[i].z
			count += 1
		end
	end
	if count > 0 then ccx, ccy, czz = ccx / count, ccy / count, czz / count end
	self.restCentroid = { ccx, ccy, czz }

	-- Signed volume from triangle fan around centroid
	local volume = 0
	for ti = 1, #triangles do
		local tri = triangles[ti]
		local a = nodes[tri.a + 1]
		local b = nodes[tri.b + 1]
		local c = nodes[tri.c + 1]
		if a and b and c then
			local ax, ay, az = a.x - ccx, a.y - ccy, a.z - czz
			local bx, by, bz = b.x - ccx, b.y - ccy, b.z - czz
			local cx2, cy2, cz2 = c.x - ccx, c.y - ccy, c.z - czz
			volume += (ax * (by * cz2 - bz * cy2) + ay * (bz * cx2 - bx * cz2) + az * (bx * cy2 - by * cx2)) / 6
		end
	end
	self.restVolume = volume
end

function World:setDrivetrainLayout(layout)
	if type(layout) == "string" then
		layout = ({ RWD = 0, FWD = 1, AWD = 2, rwd = 0, fwd = 1, awd = 2 })[layout] or 0
	end
	if type(layout) ~= "number" or layout ~= layout then layout = 0 end
	self.driveLayout = math.clamp(math.floor(layout), 0, 2)
end

function World:setControls(controls)
	controls = controls or {}
	local function bounded(value, low, high)
		if type(value) ~= "number" or value ~= value then value = 0 end
		return math.clamp(value, low, high)
	end
	self.controls.steering = bounded(controls.steering, -1, 1)
	self.controls.throttle = bounded(controls.throttle, 0, 1)
	self.controls.brake = bounded(controls.brake, 0, 1)
	self.controls.clutch = bounded(controls.clutch, 0, 1)
	self.controls.handbrake = controls.handbrake == true
	self.controls.gear_up = controls.gear_up == true
	self.controls.gear_down = controls.gear_down == true
	self.controls.selector_up = controls.selector_up == true
	self.controls.selector_down = controls.selector_down == true
	self.controls.manual_shift_up = controls.manual_shift_up == true
	self.controls.manual_shift_down = controls.manual_shift_down == true
	self.controls.engine_on = controls.engine_on == true
	local stage = if type(controls.engine_stage) == "number" then controls.engine_stage else 0
	self.controls.engine_stage = math.clamp(math.floor(stage), 0, 2)
end

-- Resolve the nearest authored road segment for wheel contact. Returning
-- scalar properties avoids allocating a contact object in the fixed-step loop.
function World:surfacePropertiesAt(x, z)
	local friction = self.groundFriction
	local roughness = self.surfaceRoughness
	local moisture = self.surfaceMoisture
	local compactness = self.surfaceCompactness
	local surfaceId = self.surfacePresetIndex
	local nearestDistance = math.huge
	for _, road in self.roadSurfaces do
		local points = road.points
		local halfWidth = road.width * 0.5
		for i = 1, #points - 1 do
			local ax, az = points[i][1], points[i][2]
			local bx, bz = points[i + 1][1], points[i + 1][2]
			local dx, dz = bx - ax, bz - az
			local lengthSquared = dx * dx + dz * dz
			local t = if lengthSquared > 1e-12 then math.clamp(((x - ax) * dx + (z - az) * dz) / lengthSquared, 0, 1) else 0
			local px, pz = ax + dx * t, az + dz * t
			local distanceSquared = (x - px) * (x - px) + (z - pz) * (z - pz)
			if distanceSquared <= halfWidth * halfWidth and distanceSquared < nearestDistance then
				nearestDistance = distanceSquared
				friction = road.friction
				roughness = road.roughness
				moisture = road.moisture
				compactness = road.compactness
				surfaceId = road.surfaceId
			end
		end
	end
	return friction, roughness, moisture, compactness, surfaceId
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

-- Internal simulation methods are installed by focused subsystem modules.
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
	self:_updateBeamMaterialState(self.fixedDt)

	-- 6. Aerodynamic drag
	self:_applyDrag()

	-- 7. Telemetry
	self:_updateTelemetry()
end

-- Public step: accumulate dt and run fixed substeps.
function World:step(elapsed)
	local frameDt = if type(elapsed) == "number" and elapsed == elapsed then math.clamp(elapsed, 0, 0.25) else 0
	self.accumulator += frameDt
	local steps = 0
	while self.accumulator >= self.fixedDt and steps < self.maxSubsteps do
		self:stepFixed()
		self.time += self.fixedDt
		self.accumulator -= self.fixedDt
		steps += 1
	end
	-- Keep at most one fixed step of debt after a long frame. This avoids a
	-- slow-motion spiral while preserving deterministic catch-up behavior.
	if steps == self.maxSubsteps then
		self.accumulator = math.min(self.accumulator, self.fixedDt)
	end
end

return World
