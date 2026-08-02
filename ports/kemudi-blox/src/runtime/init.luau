--[[
	Server runtime — runs the physics simulation on Heartbeat.
	Owns the authoritative simulation state. Client receives snapshots only.
	No Roblox API in the core modules — this file is the only Roblox-dependent code.
--]]

local RunService = game:GetService("RunService")
local Physics = require(script.Parent.Parent.Parent.Parent.KemudiEngine) -- src → physics
local Contract = require(script.Parent.Parent.Parent.Parent.KemudiEngine.contract) -- adjust path

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
	self.node_parts = {}        -- map: nodeIndex → Part
	self.beam_constraints = {}  -- map: beamIndex → Beam (Roblox Instance)
	self.active = false
	self.heartbeat_conn = nil

	-- Vehicle config
	self.suspension_config = config.suspension_config
	self.steering_config = config.steering_config
	self.tire_compounds = config.tire_compounds
	self.vehicle_mass = config.vehicle_mass or 1500

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
	self.node_parts = {}
	self.beam_constraints = {}

	-- Add nodes from parts
	for i, part in model_parts do
		local pos = part.Position
		local mass = part:GetMass()
		local fixed = part.Anchored
		local id = i - 1 -- 0-indexed
		self.world:addNode(id, pos.X, pos.Y, pos.Z, mass, fixed)
		self.node_parts[id] = part
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
	self.beam_constraints = {}
	self.root_part = nil
	self.owner = nil
end

-- ============================================================
-- Simulation tick
-- ============================================================

function Runtime:update(dt)
	if not self.active then return end

	-- Step physics
	self.world:step(dt)

	-- Apply positions to Parts
	for node_idx, part in self.node_parts do
		local node = self.world.nodes[node_idx + 1] -- 0-indexed → 1-indexed
		if node and not node.fixed then
			part.CFrame = CFrame.new(node.x, node.y, node.z)
			-- Optional: set velocity for network interpolation
		end
	end
end

-- ============================================================
-- Input handling (called by owner's client via RemoteEvent)
-- ============================================================

function Runtime:setControls(controls)
	self.world.controls.steering = controls.steering or 0
	self.world.controls.throttle = controls.throttle or 0
	self.world.controls.brake = controls.brake or 0
	self.world.controls.clutch = controls.clutch or 0
	self.world.controls.handbrake = controls.handbrake or false
	self.world.controls.gear_up = controls.gear_up or false
	self.world.controls.gear_down = controls.gear_down or false
	self.world.controls.engine_on = controls.engine_on or false
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
