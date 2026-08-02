--[[
	Client script — reads player input, sends to server, receives snapshots.
	Goes in StarterPlayerScripts.
--]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local UserInputService = game:GetService("UserInputService")
local RunService = game:GetService("RunService")
local Players = game:GetService("Players")

-- Remote events
local ControlsEvent = ReplicatedStorage:WaitForChild("KemudiControls")
local SnapshotEvent = ReplicatedStorage:WaitForChild("KemudiSnapshot")
local TelemetryEvent = ReplicatedStorage:WaitForChild("KemudiTelemetry")
local SpawnEvent = ReplicatedStorage:WaitForChild("KemudiSpawn")
local DespawnEvent = ReplicatedStorage:WaitForChild("KemudiDespawn")
local Engine = ReplicatedStorage:WaitForChild("KemudiEngine")
local Chassis = require(Engine:WaitForChild("runtime"):WaitForChild("chassis"))

-- ============================================================
-- Input state
-- ============================================================

local input = {
	steering = 0,
	throttle = 0,
	brake = 0,
	clutch = 0,
	handbrake = false,
	gear_up = false,
	gear_down = false,
	engine_on = false,
}

-- Key bindings
local KEYMAP = {
	[Enum.KeyCode.W] = "throttle",
	[Enum.KeyCode.S] = "brake",
	[Enum.KeyCode.A] = "steer_left",
	[Enum.KeyCode.D] = "steer_right",
	[Enum.KeyCode.Space] = "handbrake",
	[Enum.KeyCode.E] = "gear_up",
	[Enum.KeyCode.Q] = "gear_down",
	[Enum.KeyCode.F] = "engine_on",
}

-- ============================================================
-- Input processing
-- ============================================================

local keys_held = {}

UserInputService.InputBegan:Connect(function(gameInput, processed)
	if processed then return end
	if gameInput.UserInputType == Enum.UserInputType.Keyboard then
		keys_held[gameInput.KeyCode] = true
	end
end)

UserInputService.InputEnded:Connect(function(gameInput)
	if gameInput.UserInputType == Enum.UserInputType.Keyboard then
		keys_held[gameInput.KeyCode] = false
	end
end)

local function processInput(dt)
	-- Throttle/brake (digital → analog with decay)
	local accel = keys_held[Enum.KeyCode.W] and 1 or 0
	local brake = keys_held[Enum.KeyCode.S] and 1 or 0

	-- Steering (digital with smoothing)
	local steer_target = 0
	if keys_held[Enum.KeyCode.A] then steer_target = -1 end
	if keys_held[Enum.KeyCode.D] then steer_target = 1 end

	-- Smooth steering
	local steer_speed = 3.0
	local diff = steer_target - input.steering
	if math.abs(diff) < steer_speed * dt then
		input.steering = steer_target
	else
		input.steering = input.steering + math.sign(diff) * steer_speed * dt
	end

	input.throttle = accel
	input.brake = brake
	input.handbrake = keys_held[Enum.KeyCode.Space] or false
	input.engine_on = keys_held[Enum.KeyCode.F] or false

	-- Edge-triggered gear shifts
	if keys_held[Enum.KeyCode.E] then
		input.gear_up = true
		keys_held[Enum.KeyCode.E] = false -- consume
	else
		input.gear_up = false
	end

	if keys_held[Enum.KeyCode.Q] then
		input.gear_down = true
		keys_held[Enum.KeyCode.Q] = false
	else
		input.gear_down = false
	end
end

-- ============================================================
-- Spawn request
-- ============================================================

local function requestSpawn()
	SpawnEvent:FireServer({
		vehicle_mass = 1500,
	})
end

local function requestDespawn()
	DespawnEvent:FireServer()
end

-- ============================================================
-- Snapshot handling
-- ============================================================

local local_player = Players.LocalPlayer
local node_parts = {} -- map: nodeIndex → Part (client-side presentation)
local latest_telemetry = nil
local NODE_NAMES = { "FL", "FR", "RL", "RR", "BodyFL", "BodyFR", "BodyRL", "BodyRR", "Root" }

local function refreshNodeParts()
	local model = workspace:FindFirstChild("Vehicle_" .. local_player.Name)
	if not model then return end
	for index, name in NODE_NAMES do
		local part = model:FindFirstChild(name, true)
		if part and part:IsA("BasePart") then
			node_parts[index - 1] = part
		end
	end
end

SnapshotEvent.OnClientEvent:Connect(function(snapshot)
	-- snapshot: flat array [time, x0, y0, z0, vx0, vy0, vz0, x1, ...]
	if not snapshot or #snapshot < 2 then return end

	local time = snapshot[1]
	local node_count = (#snapshot - 1) / 6
	if next(node_parts) == nil then refreshNodeParts() end
	local snapshot_nodes = table.create(node_count)

	for i = 0, node_count - 1 do
		local base = i * 6 + 2
		local x = snapshot[base]
		local y = snapshot[base + 1]
		local z = snapshot[base + 2]
		snapshot_nodes[i + 1] = { x = x, y = y, z = z }
	end

	local chassisFrame = Chassis.frameFromNodes(snapshot_nodes)
	for i = 0, node_count - 1 do
		local base = i * 6 + 2
		local x = snapshot[base]
		local y = snapshot[base + 1]
		local z = snapshot[base + 2]

		-- Find or create client-side node part
		local part = node_parts[i]
		if part and part.Parent then
			if chassisFrame and i >= 4 then
				local right = chassisFrame.right
				local up = chassisFrame.up
				local forward = chassisFrame.forward
				part.CFrame = CFrame.fromMatrix(
					Vector3.new(x, y, z),
					Vector3.new(right[1], right[2], right[3]),
					Vector3.new(up[1], up[2], up[3]),
					Vector3.new(-forward[1], -forward[2], -forward[3]))
			else
				part.CFrame = CFrame.new(x, y, z)
			end
		end
	end
end)

TelemetryEvent.OnClientEvent:Connect(function(telemetry)
	-- Consumers can subscribe to this adapter later; retaining the latest packet
	-- keeps the replication boundary observable without logging every frame.
	if type(telemetry) == "table" then
		latest_telemetry = telemetry
	end
end)

-- ============================================================
-- Main loop
-- ============================================================

local send_interval = 1 / 20 -- 20 Hz input rate
local send_timer = 0

RunService.Heartbeat:Connect(function(dt)
	processInput(dt)

	send_timer += dt
	if send_timer >= send_interval then
		send_timer = 0
		ControlsEvent:FireServer(input)
	end
end)

-- Auto-spawn on join
Players.LocalPlayer.CharacterAdded:Connect(function()
	task.wait(1) -- wait for character to load
	requestSpawn()
end)
if local_player.Character then
	task.defer(requestSpawn)
end

-- Cleanup on leave
game:BindToClose(function()
	requestDespawn()
end)

print("[KemudiEngine] Client runtime initialized")
