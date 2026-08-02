--[[
	Server script — manages vehicle lifecycle, receives client input, sends snapshots.
	Goes in ServerScriptService.
--]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local Players = game:GetService("Players")
local Runtime = require(ReplicatedStorage.KemudiEngine.runtime)

-- ============================================================
-- Remote events (create once)
-- ============================================================

local function getOrCreate(name, className)
	local obj = ReplicatedStorage:FindFirstChild(name)
	if obj then return obj end
	obj = Instance.new(className)
	obj.Name = name
	obj.Parent = ReplicatedStorage
	return obj
end

local ControlsEvent = getOrCreate("KemudiControls", "RemoteEvent")
local SnapshotEvent = getOrCreate("KemudiSnapshot", "RemoteEvent")
local TelemetryEvent = getOrCreate("KemudiTelemetry", "RemoteEvent")
local SpawnEvent = getOrCreate("KemudiSpawn", "RemoteEvent")
local DespawnEvent = getOrCreate("KemudiDespawn", "RemoteEvent")

-- ============================================================
-- Vehicle registry
-- ============================================================

local vehicles = {} -- playerUserId → Runtime instance

local function getPlayerVehicle(player)
	return vehicles[player.UserId]
end

-- ============================================================
-- Spawn a vehicle for a player
-- ============================================================

local function spawnVehicle(player, config)
	-- Cleanup existing
	if vehicles[player.UserId] then
		vehicles[player.UserId]:despawn()
	end

	-- Create vehicle model from config
	local model = Instance.new("Model")
	model.Name = "Vehicle_" .. player.Name

	-- For now, create a simple box vehicle (6 nodes: 4 suspension + 2 body)
	local parts = {}
	local function makePart(name, size, pos, anchored, mass)
		local part = Instance.new("Part")
		part.Name = name
		part.Size = size
		part.Position = player.Character and player.Character.PrimaryPart.Position + pos or pos
		part.Anchored = anchored
		part.CanCollide = false -- physics handles collision
		part.Parent = model
		table.insert(parts, part)
		return part
	end

	-- 4 suspension nodes
	makePart("FL", Vector3.new(0.2, 0.2, 0.2), Vector3.new(-0.8, 0.3, -1.2), false, 15)
	makePart("FR", Vector3.new(0.2, 0.2, 0.2), Vector3.new(0.8, 0.3, -1.2), false, 15)
	makePart("RL", Vector3.new(0.2, 0.2, 0.2), Vector3.new(-0.8, 0.3, 1.2), false, 15)
	makePart("RR", Vector3.new(0.2, 0.2, 0.2), Vector3.new(0.8, 0.3, 1.2), false, 15)

	-- 2 body nodes
	makePart("BodyFL", Vector3.new(0.3, 0.4, 0.3), Vector3.new(-0.8, 0.8, -1.0), false, 200)
	makePart("BodyFR", Vector3.new(0.3, 0.4, 0.3), Vector3.new(0.8, 0.8, -1.0), false, 200)
	makePart("BodyRL", Vector3.new(0.3, 0.4, 0.3), Vector3.new(-0.8, 0.8, 1.0), false, 200)
	makePart("BodyRR", Vector3.new(0.3, 0.4, 0.3), Vector3.new(0.8, 0.8, 1.0), false, 200)

	-- Root part
	local root = makePart("Root", Vector3.new(1.6, 0.3, 2.4), Vector3.new(0, 0.6, 0), false, 100)
	model.PrimaryPart = root

	-- Beam definitions (simplified structural frame)
	local beams = {
		{ nodeA = 0, nodeB = 4, stiffness = 50000, damping = 500, strength = 10000 }, -- FL→BodyFL
		{ nodeA = 1, nodeB = 5, stiffness = 50000, damping = 500, strength = 10000 }, -- FR→BodyFR
		{ nodeA = 2, nodeB = 6, stiffness = 50000, damping = 500, strength = 10000 }, -- RL→BodyRL
		{ nodeA = 3, nodeB = 7, stiffness = 50000, damping = 500, strength = 10000 }, -- RR→BodyRR
		{ nodeA = 4, nodeB = 5, stiffness = 80000, damping = 800, strength = 20000 }, -- cross body
		{ nodeA = 6, nodeB = 7, stiffness = 80000, damping = 800, strength = 20000 }, -- cross body
		{ nodeA = 4, nodeB = 6, stiffness = 60000, damping = 600, strength = 15000 }, -- side
		{ nodeA = 5, nodeB = 7, stiffness = 60000, damping = 600, strength = 15000 }, -- side
	}

	model.Parent = workspace

	local rt = Runtime.new(config or {})
	rt:spawn(player, parts, beams, root)
	vehicles[player.UserId] = rt

	return rt
end

-- ============================================================
-- Input handling
-- ============================================================

ControlsEvent.OnServerEvent:Connect(function(player, controls)
	local rt = getPlayerVehicle(player)
	if not rt then return end

	-- Validate controls
	local safe = {
		steering = math.clamp(tonumber(controls.steering) or 0, -1, 1),
		throttle = math.clamp(tonumber(controls.throttle) or 0, 0, 1),
		brake = math.clamp(tonumber(controls.brake) or 0, 0, 1),
		clutch = math.clamp(tonumber(controls.clutch) or 0, 0, 1),
		handbrake = controls.handbrake == true,
		gear_up = controls.gear_up == true,
		gear_down = controls.gear_down == true,
		engine_on = controls.engine_on == true,
	}
	rt:setControls(safe)
end)

-- ============================================================
-- Spawn/despawn
-- ============================================================

SpawnEvent.OnServerEvent:Connect(function(player, config)
	spawnVehicle(player, config)
end)

DespawnEvent.OnServerEvent:Connect(function(player)
	local rt = getPlayerVehicle(player)
	if rt then
		rt:despawn()
		vehicles[player.UserId] = nil
	end
end)

-- ============================================================
-- Cleanup on leave
-- ============================================================

Players.PlayerRemoving:Connect(function(player)
	local rt = getPlayerVehicle(player)
	if rt then
		rt:despawn()
		vehicles[player.UserId] = nil
	end
end)

print("[KemudiEngine] Server runtime initialized")
