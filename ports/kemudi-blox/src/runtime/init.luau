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
	self.model = nil
	self.node_parts = {}        -- map: nodeIndex → Part
	self.oriented_parts = {}    -- map: nodeIndex → body/root presentation Part
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
	self.vehicle_mass = config.vehicle_mass or 1500
	self.drive_layout = config.drive_layout or 0

	return self
end

-- ============================================================
-- Vehicle lifecycle
-- ============================================================

-- Spawn: create the vehicle from a data definition.
-- model_parts: array of Part instances (one per node)
-- beam_defs: array of { nodeA, nodeB, stiffness, damping, strength }
function Runtime:spawn(owner, model_parts, beam_defs, root_part)
	self.owner = owner
	self.root_part = root_part
	self.model = if root_part then root_part.Parent else nil
	self.node_parts = {}
	self.oriented_parts = {}
	self.beam_constraints = {}
	self.control_age = 0
	self.snapshot_timer = 0
	self.telemetry_timer = 0

	-- Add nodes from parts
	for i, part in model_parts do
		local pos = part.Position
		local authoredMass = part:GetAttribute("KemudiMassKg")
		local mass = if type(authoredMass) == "number" and authoredMass > 0 then authoredMass else part:GetMass()
		local fixed = part:GetAttribute("KemudiFixed") == true
		local id = i - 1 -- 0-indexed
		self.world:addNode(id, pos.X, pos.Y, pos.Z, mass, fixed)
		self.node_parts[id] = part
		self.oriented_parts[id] = part:GetAttribute("KemudiOriented") == true or id >= 4
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
		self.vehicle_mass
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
	self.beam_constraints = {}
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
	if self.control_age > 0.25 then
		-- Dead-man behavior: stop propulsion and request braking when input stops.
		self.world:setControls({ brake = 1 })
	end

	-- Step physics
	self.world:step(dt)

	-- Apply positions and the emergent chassis orientation to Parts.
	local chassisFrame = Chassis.frameFromNodes(self.world.nodes)
	for node_idx, part in self.node_parts do
		local node = self.world.nodes[node_idx + 1] -- 0-indexed → 1-indexed
		if node and not node.fixed then
			if chassisFrame and self.oriented_parts[node_idx] then
				local right = chassisFrame.right
				local up = chassisFrame.up
				local forward = chassisFrame.forward
				part.CFrame = CFrame.fromMatrix(
					Vector3.new(node.x, node.y, node.z),
					Vector3.new(right[1], right[2], right[3]),
					Vector3.new(up[1], up[2], up[3]),
					Vector3.new(-forward[1], -forward[2], -forward[3]))
			else
				part.CFrame = CFrame.new(node.x, node.y, node.z)
			end
			-- Optional: set velocity for network interpolation
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
