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
local Brakes = if script then require(script.Parent.vehicle.brakes) else require("../vehicle/brakes")
local Turbo = if script then require(script.Parent.vehicle.turbo) else require("../vehicle/turbo")
local WheelDynamics = if script then require(script.Parent.vehicle.wheel_dynamics) else require("../vehicle/wheel_dynamics")
local SuspensionKinematics = if script then require(script.Parent.vehicle.suspension_kinematics) else require("../vehicle/suspension_kinematics")
local EngineBreathing = if script then require(script.Parent.vehicle.engine_breathing) else require("../vehicle/engine_breathing")

local Integration = if script then require(script.integration) else require("./integration")
local Constraints = if script then require(script.constraints) else require("./constraints")
local Collision = if script then require(script.collision) else require("./collision")
local Aerodynamics = if script then require(script.aerodynamics) else require("./aerodynamics")
local Telemetry = if script then require(script.telemetry) else require("./telemetry")
local Vehicle = if script then require(script.vehicle) else require("./vehicle")
local Angular = if script then require(script.angular) else require("./angular")
local Bushings = if script then require(script.bushings) else require("./bushings")
local TerrainDeform = if script then require(script.terrain_deform) else require("./terrain_deform")
local VehicleCollision = if script then require(script.vehicle_collision) else require("./vehicle_collision")
local SolverHarden = if script then require(script.solver_harden) else require("./solver_harden")
local Energy = if script then require(script.energy) else require("./energy")
local ContactModel = if script then require(script.contact_model) else require("./contact_model")

local World = {}
World.__index = World

Integration.install(World, Math)
Constraints.install(World, Math)
Collision.install(World, Contract, Math)
Aerodynamics.install(World)
Telemetry.install(World, Contract, Safety)
Vehicle.install(World, Suspension, Tires, Drivetrain, Safety, Math, EngineThermal, Brakes, Turbo, WheelDynamics, SuspensionKinematics, EngineBreathing)
Angular.install(World, Math)
Bushings.install(World, Math)
TerrainDeform.install(World, Contract, Math)
VehicleCollision.install(World, Math)
SolverHarden.install(World, Math)
Energy.install(World, Math)
ContactModel.install(World, Math)

local function makeTelemetry()
	local t = table.create(Contract.TELEMETRY_LEN, 0)
	t[Contract.T.DERATE + 1] = 1.0
	return t
end

local function peakVerticalVelocity(nodes)
	local peak = 0
	for _, node in nodes do
		if not node.fixed then peak = math.max(peak, math.abs(node.vy or 0)) end
	end
	return peak
end

local function verticalForceSummary(nodes)
	local total, peak = 0, 0
	for _, node in nodes do
		if not node.fixed then
			local fy = node.fy or 0
			total += fy
			peak = math.max(peak, math.abs(fy))
		end
	end
	return total, peak
end

function World.new()
	local self = setmetatable({}, World)
	self.nodes = {}
	self.beams = {}
	self.triangles = {}
	self.gravity = Contract.DEFAULT_GRAVITY
	self.time = 0
	self.airDensity = nil
	self.dragCoefficient = nil
	self.frontalArea = nil
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
	self.flightDiagnosticEnabled = true
	self.flightDiagnosticReported = false

	-- Volume preservation state
	self.restVolume = 0
	self.restCentroid = { 0, 0, 0 }
	self.volumeLambda = 0
	self.volumeCompliance = nil
	self.bodyAttachmentSoftness = nil

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

	-- Brake system (per-wheel thermal model)
	self.brake_states = nil
	self.brake_config = nil
	-- Turbo / supercharger
	self.turbo_state = nil
	self.turbo_config = nil

	return self
end

-- Set up a full vehicle with 4-wheel suspension, tires, drivetrain, safety.
function World:setupVehicle(suspension_cfg, steering_cfg, tire_compounds, vehicle_mass, drivetrain_config, simulation_config)
	assert(type(vehicle_mass) == "number" and vehicle_mass > 0, "vehicle_mass must come from validated asset mass proxies")
	assert(type(simulation_config) == "table", "simulation_config must come from the authored vehicle asset")
	assert(type(simulation_config.aero) == "table", "vehicle asset is missing Simulation.Aero")
	assert(type(simulation_config.brakes) == "table", "vehicle asset is missing Simulation.Brakes")
	assert(type(simulation_config.wheel_dynamics) == "table", "vehicle asset is missing Simulation.WheelDynamics")
	assert(type(simulation_config.suspension_kinematics) == "table", "vehicle asset is missing Simulation.SuspensionKinematics")
	assert(type(simulation_config.engine_breathing) == "table", "vehicle asset is missing Simulation.EngineBreathing")
	assert(type(simulation_config.angular) == "table" and type(simulation_config.angular.inertia) == "table", "vehicle asset is missing Simulation.Angular")
	assert(type(simulation_config.solver) == "table", "vehicle asset is missing Simulation.Solver")
	assert(type(simulation_config.environment) == "table", "vehicle asset is missing Simulation.Environment")
	assert(type(simulation_config.contact) == "table", "vehicle asset is missing Simulation.Contact")
	assert(type(simulation_config.terrain) == "table", "vehicle asset is missing Simulation.Terrain")
	assert(type(simulation_config.collision) == "table", "vehicle asset is missing Simulation.Collision")
	assert(type(simulation_config.bushings) == "table", "vehicle asset is missing Simulation.Bushings")
	self.suspension_config = suspension_cfg or Suspension.newConfig()
	self.steering_config = steering_cfg or Suspension.newSteeringConfig()
	self.vehicle_mass = vehicle_mass
	self.simulation_config = simulation_config
	self.airDensity = simulation_config.aero.air_density
	self.frontalArea = simulation_config.aero.frontal_area
	self.dragCoefficient = simulation_config.aero.cd
	self.aeroCd = simulation_config.aero.cd
	self.aeroCl = simulation_config.aero.cl
	self.aeroCs = simulation_config.aero.cs
	self.aeroCm = simulation_config.aero.cm
	self.aeroWingCl = simulation_config.aero.wing_cl
	self.aeroWingArea = simulation_config.aero.wing_area
	self.aeroNodeDrag = simulation_config.aero.node_drag
	self.aeroGroundRefHeight = simulation_config.aero.ground_ref_height
	self.aeroPitchBrakeCoeff = simulation_config.aero.pitch_brake_coeff
	self.aeroPitchPowerCoeff = simulation_config.aero.pitch_power_coeff
	self.aeroGroundEffectMin = simulation_config.aero.ground_effect_min
	self.aeroGroundEffectMax = simulation_config.aero.ground_effect_max

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

	-- Brake system
	self.brake_config = Brakes.newConfig(simulation_config.brakes)
	self.brake_states = {}
	for _ = 1, 4 do
		table.insert(self.brake_states, Brakes.newState())
	end

	-- Turbo (disabled by default; vehicle config can override)
	self.turbo_config = Turbo.newConfig(simulation_config.turbo)
	self.turbo_state = Turbo.newState()

	-- Wheel dynamics (per-wheel rotational inertia)
	self.wheel_dynamics_configs = {}
	self.wheel_dynamics_states = {}
	for i = 1, 4 do
		local wheel_cfg = simulation_config.wheel_dynamics.wheels and simulation_config.wheel_dynamics.wheels[i]
		assert(type(wheel_cfg) == "table", "vehicle asset is missing Simulation.WheelDynamics wheel " .. i)
		local config = WheelDynamics.newConfig(wheel_cfg)
		local state = WheelDynamics.newState()
		state.tire_radius = config.tire_radius
		state.inertia = WheelDynamics.computeInertia(config)
		table.insert(self.wheel_dynamics_configs, config)
		table.insert(self.wheel_dynamics_states, state)
	end

	-- Suspension kinematics (camber, toe, roll center, anti-dive/squat)
	self.suspension_kinematics_config = SuspensionKinematics.newConfig(simulation_config.suspension_kinematics)

	-- Engine breathing (volumetric efficiency, altitude, accessories)
	self.engine_breathing_config = EngineBreathing.newConfig(simulation_config.engine_breathing)
	self.engine_breathing_state = EngineBreathing.newState()

	-- Angular dynamics initialization
	self.centerOfMass = { 0, 0, 0 }
	self.angularVelocity = { 0, 0, 0 }
	self.inertia = simulation_config.angular.inertia
	self.inertia_override = simulation_config.angular.inertia
	self.forwardVector = { 0, 0, -1 }
	self.rightVector = { 1, 0, 0 }
	self.restCoM = { 0, 0, 0 }
	self.restRelativePositions = {}
	self.shapeMatchingStiffness = simulation_config.angular.shape_matching_stiffness
	self.angularDamping = simulation_config.angular.angular_damping
	self.volumeCompliance = simulation_config.solver.volume_compliance
	self.bodyAttachmentSoftness = simulation_config.solver.body_attachment_softness
	self.solverIterations = simulation_config.solver.iterations
	self.maxSolverIterations = simulation_config.solver.max_iterations
	self.velocityDamping = simulation_config.solver.velocity_damping
	self.ambient_temperature = simulation_config.environment.ambient_temperature
	self.contact_patch_area = simulation_config.contact.tire_contact_patch_area
	self.surfaceHardness = simulation_config.terrain.surface_hardness
	self.rutRecoveryRate = simulation_config.terrain.rut_recovery_rate
	self.maxRutDepth = simulation_config.terrain.max_rut_depth
	self.spreadFactor = simulation_config.terrain.spread_factor
	self.recoveryDepthDamping = simulation_config.terrain.recovery_depth_damping
	self.resistanceMax = simulation_config.terrain.resistance_max
	self.cellSize = simulation_config.collision.cell_size
	self.gridExtent = simulation_config.collision.grid_extent
	self.collision_contact_radius = simulation_config.collision.contact_radius
	self:setupContactModel(simulation_config.contact)
	self:setupTerrainDeformation()
	self:setupVehicleCollision()
	self:setupEnergyTracking()
	for _, bushing in ipairs(simulation_config.bushings) do
		self:addBushing(bushing.node_a, bushing.node_b, bushing)
	end
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

	-- 0. Compute inertia tensor (every 4 steps for perf)
	if self.stepCount == nil then self.stepCount = 0 end
	self.stepCount += 1
	if self.stepCount % 4 == 0 and self._computeInertiaTensor then
		self:_computeInertiaTensor()
	end

	-- 1. Update vehicle systems (suspension, tires, drivetrain, safety, brakes, turbo, wheel dynamics, engine breathing)
	self:_updateVehicleSystems()

	-- 2. Gravity and downforce
	self:_applyForces()
	local forceY, peakForceY = verticalForceSummary(nodes)

	-- 3. Angular velocity integration (applies rotation to nodes)
	if self._computeAngularVelocity and self._applyAngularMotion then
		self:_computeAngularVelocity()
		self:_applyAngularMotion()
	end

	-- 4. Integrate velocities
	self:_integrateVelocities(self.fixedDt)
	local integratedPeakVy = peakVerticalVelocity(nodes)

	-- Save unconstrained positions
	for i = 1, count do
		local n = nodes[i]
		local sp = self.constraintStartPositions[i]
		sp[1] = n.x
		sp[2] = n.y
		sp[3] = n.z
	end

	-- 5. XPBD constraint solver
	self:_solveXPBD()
	local constrainedPeakVy = peakVerticalVelocity(nodes)

	-- 6. Bushings compliance forces
	if self._updateBushings then
		self:_updateBushings(self.fixedDt)
	end

	-- 7. Terrain + static collision
	self:_collideWithTerrain()
	self:_collideWithStaticGeometry()
	local collisionPeakVy = peakVerticalVelocity(nodes)
	self:_updateBeamMaterialState(self.fixedDt)

	-- 7b. Contact model (spring-damper terrain interaction)
	if self._applyContactForces then
		self:_applyContactForces(self.fixedDt)
	end

	-- 8. Terrain deformation (write wheel pressure into rut grid)
	if self._updateTerrainDeformation then
		self:_updateTerrainDeformation(self.fixedDt)
	end
	if self._recoverTerrainDeformation then
		self:_recoverTerrainDeformation(self.fixedDt)
	end

	-- 9. Aerodynamic forces (full model: drag, lift, pitch, yaw)
	self:_applyDrag()

	-- 10. Vehicle-to-vehicle collision
	if self._collideWithOtherVehicles then
		self:_collideWithOtherVehicles(self.fixedDt)
	end

	-- 11. Energy tracking
	if self._computeEnergy then
		self:_computeEnergy()
	end

	-- 12. Rigid chassis shape matching (when grounded)
	if self._shapeMatchingProjection and self.suspension_states then
		local groundedCount = 0
		for i = 1, math.min(#self.suspension_states, 4) do
			if not self.suspension_states[i].is_airborne then groundedCount += 1 end
		end
		if groundedCount >= 2 then
			self:_shapeMatchingProjection(self.shapeMatchingStiffness)
		end
	end

	-- 13. Velocity damping (anti-explosion)
	if self._applyVelocityDamping then
		self:_applyVelocityDamping(self.fixedDt)
	end

	-- 14. Anti-tunnel clamping
	if self._antiTunnelClamp then
		self:_antiTunnelClamp()
	end
	if self._enforceParkedRestState then
		self:_enforceParkedRestState()
	end
	local finalPeakVy = peakVerticalVelocity(nodes)
	if self.flightDiagnosticEnabled then
		if finalPeakVy > 15 and not self.flightDiagnosticReported then
			local minY, maxY = math.huge, -math.huge
			for _, node in nodes do
				if not node.fixed then minY = math.min(minY, node.y); maxY = math.max(maxY, node.y) end
			end
			local loads, airborne = {}, {}
			for i = 1, math.min(#(self.suspension_states or {}), 4) do
				local state = self.suspension_states[i]
				loads[i] = string.format("%.0f", state.load or 0)
				airborne[i] = if state.is_airborne then "1" else "0"
			end
			local drivetrain = self.drivetrain
			local transmission = drivetrain and drivetrain.transmission
			local engine = drivetrain and drivetrain.engine
			local wheelSpeeds = {}
			for i = 1, math.min(#(self.wheel_dynamics_states or {}), 4) do
				wheelSpeeds[i] = string.format("%.1f", self.wheel_dynamics_states[i].angular_velocity or 0)
			end
			local broken = 0
			for _, beam in beams do if beam.broken then broken += 1 end end
			print(string.format(
				"[KemudiFlight] dt=%.6f vy=%.2f stages=%.2f/%.2f/%.2f/%.2f y=%.2f..%.2f forceY=%.0f peakFy=%.0f loads=%s airborne=%s wheels=%s rpm=%.0f gear=%s driveTorque=%.0f impact=%.3f broken=%d",
				self.fixedDt, finalPeakVy, integratedPeakVy, constrainedPeakVy, collisionPeakVy, finalPeakVy,
				minY, maxY, forceY, peakForceY, table.concat(loads, ","), table.concat(airborne, ","),
				table.concat(wheelSpeeds, ","), engine and engine.rpm or 0, transmission and tostring(transmission.current_gear) or "?",
				drivetrain and drivetrain.last_drive_torque or 0, self.telemetry[Contract.T.IMPACT_SEVERITY + 1] or 0, broken
			))
			self.flightDiagnosticReported = true
		elseif finalPeakVy < 5 then
			self.flightDiagnosticReported = false
		end
	end

	-- 15. Telemetry
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
