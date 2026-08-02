--[[
	Server runtime — runs the physics simulation on Heartbeat.
	Owns the authoritative simulation state. Client receives snapshots only.
	No Roblox API in the core modules — this file is the only Roblox-dependent code.
--]]

local RunService = game:GetService("RunService")
local Physics = require(script.Parent.physics)
local Contract = require(script.Parent.contract)
local Chassis = require(script.chassis)

local Runtime = {}
Runtime.__index = Runtime

local function frameToCFrame(frame, scale)
	local right = frame.right
	local up = frame.up
	local forward = frame.forward
	local position = frame.position
	return CFrame.fromMatrix(
		Vector3.new(position[1] * scale, position[2] * scale, position[3] * scale),
		Vector3.new(right[1], right[2], right[3]),
		Vector3.new(up[1], up[2], up[3]),
		Vector3.new(-forward[1], -forward[2], -forward[3]))
end

-- ============================================================
-- Vehicle instance — binds physics World to Roblox workspace
-- ============================================================

local function makeVehicleId()
	return string.format("vehicle_%d_%d", os.clock() * 1000, math.random(1000, 9999))
end

function Runtime.new(config)
	config = config or {}
	local self = setmetatable({}, Runtime)

	self.world = Physics.new()  -- returns the World module
	self.id = makeVehicleId()
	self.owner = nil            -- Player who owns this vehicle
	self.root_part = nil        -- PrimaryPart of the vehicle model
	self.driver_seat = nil      -- authored Seat used for server-authoritative entry
	self.driver_seat_binding = nil
	self.engine_state = "OFF"
	self.engine_on = false
	self.model = nil
	self.node_parts = {}        -- map: nodeIndex → Part
	self.oriented_parts = {}    -- map: nodeIndex → body/root presentation Part
	self.render_bindings = {}   -- render BaseParts bound to the solver chassis frame
	self.wheel_bindings = {}    -- authored wheel visuals bound to wheel spin/steering
	self.wheel_spin = { 0, 0, 0, 0 }
	self.beam_constraints = {}  -- map: beamIndex → Beam (Roblox Instance)
	self.active = false
	self.heartbeat_conn = nil
	self.control_age = 0
	self.snapshot_timer = 0
	self.telemetry_timer = 0
	self.snapshot_interval = config.snapshot_interval or (1 / 20)
	self.telemetry_interval = config.telemetry_interval or (1 / 20)
	self.on_snapshot = config.on_snapshot
	self.on_telemetry = config.on_telemetry

	-- Vehicle config
	self.suspension_config = config.suspension_config
	self.steering_config = config.steering_config
	self.tire_compounds = config.tire_compounds
	self.drivetrain_config = config.drivetrain_config
	self.vehicle_mass = config.vehicle_mass
	self.drive_layout = config.drive_layout
	self.studs_per_meter = config.studs_per_meter
	self.suspension_config = config.suspension_config

	return self
end

-- ============================================================
-- Vehicle lifecycle
-- ============================================================

-- Spawn: create the vehicle from a data definition.
-- node_defs: array of { part, mass, fixed } definitions from the asset loader
-- beam_defs: array of { nodeA, nodeB, stiffness, damping, strength }
-- render_parts: anchored visual BaseParts bound to the emergent chassis frame
function Runtime:spawn(owner, node_defs, beam_defs, root_part, render_parts, wheels, driver_seat)
	self.owner = owner
	self.root_part = root_part
	self.driver_seat = driver_seat
	self.driver_seat_binding = nil
	self.engine_state = "OFF"
	self.engine_on = false
	self.model = if root_part then root_part:FindFirstAncestorOfClass("Model") else nil
	self.node_parts = {}
	self.oriented_parts = {}
	self.render_bindings = {}
	self.beam_constraints = {}
	self.wheel_bindings = {}
	self.wheel_spin = { 0, 0, 0, 0 }
	self.control_age = 0
	self.snapshot_timer = 0
	self.telemetry_timer = 0

	assert(type(self.vehicle_mass) == "number" and self.vehicle_mass > 0, "vehicle mass must be derived from an authored asset")
	assert(type(self.studs_per_meter) == "number" and self.studs_per_meter > 0, "asset must define StudsPerMeter")
	assert(driver_seat and driver_seat:IsA("Seat"), "asset must define an authored DriverSeat")

	-- Add nodes from validated asset definitions.
	for i, node_def in node_defs do
		local part = node_def.part
		local pos = part.Position
		local mass = node_def.mass
		local fixed = node_def.fixed == true
		local id = i - 1 -- 0-indexed
		self.world:addNode(id, pos.X / self.studs_per_meter, pos.Y / self.studs_per_meter, pos.Z / self.studs_per_meter, mass, fixed)
		self.node_parts[id] = part
		self.oriented_parts[id] = part:GetAttribute("KemudiOriented") == true or id >= 4
	end

	local initialChassisFrame = Chassis.frameFromNodes(self.world.nodes)
	if initialChassisFrame and render_parts then
		local initialCFrame = frameToCFrame(initialChassisFrame, self.studs_per_meter)
		self.driver_seat_binding = initialCFrame:ToObjectSpace(driver_seat.CFrame)
		for _, part in render_parts do
			table.insert(self.render_bindings, {
				part = part,
				offset = initialCFrame:ToObjectSpace(part.CFrame),
			})
		end
		local wheelIds = { "FL", "FR", "RL", "RR" }
		local seenVisuals = {}
		for index, wheelId in wheelIds do
			local wheel = wheels and wheels[wheelId]
			local visual = if wheel then wheel.visual else nil
			if visual then
				local existing = seenVisuals[visual]
				if existing then
					table.insert(existing.indices, index)
					local node = self.world.nodes[index]
					local nodePosition = Vector3.new(node.x * self.studs_per_meter, node.y * self.studs_per_meter, node.z * self.studs_per_meter)
					table.insert(existing.node_local_positions, initialCFrame:PointToObjectSpace(nodePosition))
				else
					local node = self.world.nodes[index]
					local nodePosition = Vector3.new(node.x * self.studs_per_meter, node.y * self.studs_per_meter, node.z * self.studs_per_meter)
					local binding = {
						indices = { index },
						visual = visual,
						offset = initialCFrame:ToObjectSpace(visual.CFrame),
						node_local_positions = { initialCFrame:PointToObjectSpace(nodePosition) },
					}
					seenVisuals[visual] = binding
					table.insert(self.wheel_bindings, binding)
				end
			end
		end
	end

	-- Add beams
	if beam_defs then
		for i, beam in beam_defs do
			local id = i - 1
			self.world:addBeam(id, beam.nodeA, beam.nodeB,
				beam.stiffness, beam.damping, beam.strength)
		end
	end

	-- Setup vehicle systems
	self.world:setupVehicle(
		self.suspension_config,
		self.steering_config,
		self.tire_compounds,
		self.vehicle_mass,
		self.drivetrain_config
	)
	self.world:setDrivetrainLayout(self.drive_layout)

	-- Start simulation
	self.active = true
	self.heartbeat_conn = RunService.Heartbeat:Connect(function(dt)
		self:update(dt)
	end)
end

-- Despawn: disconnect all loops, clean up references.
function Runtime:despawn()
	self.active = false
	if self.heartbeat_conn then
		self.heartbeat_conn:Disconnect()
		self.heartbeat_conn = nil
	end
	self.node_parts = {}
	self.oriented_parts = {}
	self.render_bindings = {}
	self.beam_constraints = {}
	self.wheel_bindings = {}
	self.wheel_spin = { 0, 0, 0, 0 }
	self.driver_seat = nil
	self.driver_seat_binding = nil
	self.engine_state = "OFF"
	self.engine_on = false
	if self.model then
		self.model:Destroy()
		self.model = nil
	end
	self.root_part = nil
	self.owner = nil
end

-- ============================================================
-- Simulation tick
-- ============================================================

function Runtime:update(dt)
	if not self.active then return end
	if type(dt) ~= "number" or dt ~= dt or dt < 0 then dt = 0 end
	self.control_age += dt
	self.world.controls.engine_on = self.engine_on
	self.world.controls.engine_stage = if self.engine_state == "ACCESSORY" then 1 elseif self.engine_state == "ON" then 2 else 0
	if self.control_age > 0.25 then
		-- Dead-man behavior: stop propulsion and request braking when input stops.
		self.world:setControls({ brake = 1, engine_on = self.engine_on, engine_stage = self.world.controls.engine_stage })
	end

	-- Step physics
	self.world:step(dt)

	-- Apply positions and the emergent chassis orientation to Parts.
	local chassisFrame = Chassis.frameFromNodes(self.world.nodes)
	local chassisCFrame = if chassisFrame then frameToCFrame(chassisFrame, self.studs_per_meter) else nil
	for node_idx, part in self.node_parts do
		local node = self.world.nodes[node_idx + 1] -- 0-indexed → 1-indexed
		if node and not node.fixed then
			if chassisFrame and self.oriented_parts[node_idx] then
				local right = chassisFrame.right
				local up = chassisFrame.up
				local forward = chassisFrame.forward
				part.CFrame = CFrame.fromMatrix(
					Vector3.new(node.x * self.studs_per_meter, node.y * self.studs_per_meter, node.z * self.studs_per_meter),
					Vector3.new(right[1], right[2], right[3]),
					Vector3.new(up[1], up[2], up[3]),
					Vector3.new(-forward[1], -forward[2], -forward[3]))
			else
				part.CFrame = CFrame.new(node.x * self.studs_per_meter, node.y * self.studs_per_meter, node.z * self.studs_per_meter)
			end
			-- Optional: set velocity for network interpolation
		end
	end
	if chassisCFrame then
		if self.driver_seat and self.driver_seat_binding and self.driver_seat.Parent then
			self.driver_seat.CFrame = chassisCFrame * self.driver_seat_binding
		end
		for _, binding in self.render_bindings do
			if binding.part.Parent then
				binding.part.CFrame = chassisCFrame * binding.offset
			end
		end
		local steeringStates = self.world.steering_states
		local suspensionStates = self.world.suspension_states
		for _, binding in self.wheel_bindings do
			local nodeX, nodeY, nodeZ, angularSpeed = 0, 0, 0, 0
			local steering = 0
			for _, index in binding.indices do
				local node = self.world.nodes[index]
				local state = suspensionStates[index]
				if node and state then
					nodeX += node.x
					nodeY += node.y
					nodeZ += node.z
					angularSpeed += state.angular_speed or 0
					if index <= 2 then steering += steeringStates[index] or 0 end
				end
			end
			local count = math.max(#binding.indices, 1)
			local firstIndex = binding.indices[1]
			if binding.visual.Parent and firstIndex then
				angularSpeed /= count
				steering /= count
				self.wheel_spin[firstIndex] = math.fmod(self.wheel_spin[firstIndex] - angularSpeed * dt, math.pi * 2)
				local bodyRotation = chassisCFrame - chassisCFrame.Position
				local currentNodePosition = Vector3.new(nodeX / count * self.studs_per_meter, nodeY / count * self.studs_per_meter, nodeZ / count * self.studs_per_meter)
				local expectedNodePosition = Vector3.new(0, 0, 0)
				for _, nodeLocalPosition in binding.node_local_positions do
					expectedNodePosition += chassisCFrame:PointToWorldSpace(nodeLocalPosition)
				end
				expectedNodePosition /= math.max(#binding.node_local_positions, 1)
				local suspensionDisplacement = currentNodePosition - expectedNodePosition
				local position = chassisCFrame:PointToWorldSpace(binding.offset.Position) + suspensionDisplacement
				local wheelRotation = binding.offset - binding.offset.Position
				binding.visual.CFrame = CFrame.new(position)
					* bodyRotation
					* CFrame.Angles(0, -steering, 0)
					* wheelRotation
					* CFrame.Angles(self.wheel_spin[firstIndex], 0, 0)
			end
		end
	end

	self.snapshot_timer += dt
	if self.on_snapshot and self.snapshot_timer >= self.snapshot_interval then
		self.snapshot_timer -= self.snapshot_interval
		self.on_snapshot(self:getSnapshot())
	end
	self.telemetry_timer += dt
	if self.on_telemetry and self.telemetry_timer >= self.telemetry_interval then
		self.telemetry_timer -= self.telemetry_interval
		self.on_telemetry(self:getTelemetry())
	end
end

-- ============================================================
-- Input handling (called by owner's client via RemoteEvent)
-- ============================================================

function Runtime:setControls(controls)
	if type(controls) ~= "table" then return end
	self.control_age = 0
	self.world:setControls(controls)
	self.world.controls.engine_on = self.engine_on
	self.world.controls.engine_stage = if self.engine_state == "ACCESSORY" then 1 elseif self.engine_state == "ON" then 2 else 0
end

function Runtime:setEngineState(state)
	if state ~= "OFF" and state ~= "ACCESSORY" and state ~= "ON" then return false end
	self.engine_state = state
	self.engine_on = state == "ON"
	self.world.controls.engine_on = self.engine_on
	self.world.controls.engine_stage = if state == "ACCESSORY" then 1 elseif state == "ON" then 2 else 0
	return true
end

function Runtime:cycleEngineState()
	local nextState = if self.engine_state == "OFF" then "ACCESSORY"
		elseif self.engine_state == "ACCESSORY" then "ON"
		else "OFF"
	self:setEngineState(nextState)
	return self.engine_state
end

-- ============================================================
-- Snapshot for network replication
-- ============================================================

-- Returns a compact snapshot of node positions and velocities.
function Runtime:getSnapshot()
	local nodes = self.world.nodes
	local snapshot = table.create(#nodes * 6 + 1)
	snapshot[1] = self.world.time
	for i, node in nodes do
		local base = (i - 1) * 6 + 2
		snapshot[base] = node.x
		snapshot[base + 1] = node.y
		snapshot[base + 2] = node.z
		snapshot[base + 3] = node.vx
		snapshot[base + 4] = node.vy
		snapshot[base + 5] = node.vz
	end
	return snapshot
end

-- Returns the telemetry buffer (86 floats).
function Runtime:getTelemetry()
	return self.world.telemetry
end

return Runtime
