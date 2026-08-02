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

local function processInput()
	-- Throttle/brake (digital → analog with decay)
	local accel = keys_held[Enum.KeyCode.W] and 1 or 0
	local brake = keys_held[Enum.KeyCode.S] and 1 or 0

	-- Steering (digital with smoothing)
	local steer_target = 0
	if keys_held[Enum.KeyCode.A] then steer_target = -1 end
	if keys_held[Enum.KeyCode.D] then steer_target = 1 end

	-- Smooth steering
	local steer_speed = 3.0
	local dt = RunService.Heartbeat:Wait()
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
local node_parts = {} -- map: nodeIndex → Part (client-side predictions)

SnapshotEvent.OnClientEvent:Connect(function(snapshot)
	-- snapshot: flat array [time, x0, y0, z0, vx0, vy0, vz0, x1, ...]
	if not snapshot or #snapshot < 2 then return end

	local time = snapshot[1]
	local node_count = (#snapshot - 1) / 6

	for i = 0, node_count - 1 do
		local base = i * 6 + 2
		local x = snapshot[base]
		local y = snapshot[base + 1]
		local z = snapshot[base + 2]

		-- Find or create client-side node part
		local part = node_parts[i]
		if part then
			part.Position = Vector3.new(x, y, z)
		end
	end
end)

-- ============================================================
-- Main loop
-- ============================================================

local send_interval = 1 / 20 -- 20 Hz input rate
local send_timer = 0

RunService.Heartbeat:Connect(function(dt)
	processInput()

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

-- Cleanup on leave
game:BindToClose(function()
	requestDespawn()
end)

print("[KemudiEngine] Client runtime initialized")
