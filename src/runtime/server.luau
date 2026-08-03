--[[
	Server script — manages vehicle lifecycle, receives client input, sends snapshots.
	Goes in ServerScriptService.

	Vehicle geometry, node layout, proxy mass, wheels, and beams are authored in
	ServerStorage.KemudiVehicles. The client may request an asset ID, but cannot
	supply mass, tuning, topology, or physics configuration.
--]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local ServerStorage = game:GetService("ServerStorage")
local Players = game:GetService("Players")
local Runtime = require(ReplicatedStorage.KemudiEngine.runtime)
local VehicleLoader = require(ReplicatedStorage.KemudiEngine.runtime.vehicle_asset_loader)
local GroundZeroWorld = require(ReplicatedStorage.KemudiEngine.runtime.ground_zero_world)
local Configurator = require(ReplicatedStorage.KemudiEngine.configurator)

local groundZero = GroundZeroWorld.new({
	chunkSize = 512,
	loadRadius = 2,
	unloadRadius = 3,
	generationBudget = 2,
	maxActiveChunks = 256,
})
groundZero:start()

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
local EngineToggleEvent = getOrCreate("KemudiEngineToggle", "RemoteEvent")
local TransmissionAlertEvent = getOrCreate("KemudiTransmissionAlert", "RemoteEvent")

-- ============================================================
-- Vehicle registry
-- ============================================================

local vehicles = {} -- playerUserId → Runtime instance
local configuratorSync = Configurator.ServerSync.new(vehicles)
local vehicleTemplates = ServerStorage:WaitForChild("KemudiVehicles")

local function getPlayerVehicle(player)
	return vehicles[player.UserId]
end

local function getPlayerHumanoid(player)
	local character = player.Character
	if not character then return nil end
	local humanoid = character:FindFirstChildOfClass("Humanoid")
	if not humanoid or humanoid.Health <= 0 then return nil end
	return humanoid
end

local function configureDriverPrompt(rt, player, templateName)
	local seat = rt.driver_seat
	if not seat then return end
	local prompt = seat:FindFirstChild("KemudiEnterPrompt")
	if not prompt then
		prompt = Instance.new("ProximityPrompt")
		prompt.Name = "KemudiEnterPrompt"
		prompt.Parent = seat
	end
	prompt.ActionText = "Enter"
	prompt.ObjectText = templateName
	prompt.KeyboardKeyCode = Enum.KeyCode.E
	prompt.HoldDuration = 0
	prompt.MaxActivationDistance = 10
	prompt.RequiresLineOfSight = false
	prompt.Enabled = true

	local function updatePrompt()
		prompt.Enabled = seat.Occupant == nil
	end
	seat:GetPropertyChangedSignal("Occupant"):Connect(updatePrompt)
	prompt.Triggered:Connect(function(triggeringPlayer)
		if triggeringPlayer ~= player or getPlayerVehicle(triggeringPlayer) ~= rt then return end
		if seat.Occupant ~= nil then return end
		local humanoid = getPlayerHumanoid(triggeringPlayer)
		if humanoid then
			seat:Sit(humanoid)
		end
	end)
end

local function resolveTemplate(requestedId)
	if type(requestedId) == "string" and #requestedId > 0 and #requestedId <= 64 then
		local requested = vehicleTemplates:FindFirstChild(requestedId)
		if requested and requested:IsA("Model") then
			return requested
		end
	end

	-- A missing/invalid request selects the first authored Model. This keeps the
	-- server generic while still allowing future vehicle selection by asset ID.
	for _, candidate in ipairs(vehicleTemplates:GetChildren()) do
		if candidate:IsA("Model") then
			return candidate
		end
	end
	return nil
end

-- ============================================================
-- Spawn a vehicle for a player
-- ============================================================

local function spawnVehicle(player, requestedId)
	local template = resolveTemplate(requestedId)
	if not template then
		warn("[KemudiEngine] no authored vehicle Models found in ServerStorage.KemudiVehicles")
		return nil
	end

	if vehicles[player.UserId] then
		groundZero:unregisterVehicle(player)
		vehicles[player.UserId]:despawn()
		vehicles[player.UserId] = nil
	end

	local model = template:Clone()
	model.Name = "Vehicle_" .. player.Name
	model.Parent = workspace

	local spawnCFrame = groundZero:getSpawnCFrame()
	local authoredPivot = model:GetPivot()
	local authoredRotation = authoredPivot - authoredPivot.Position
	model:PivotTo(spawnCFrame * authoredRotation)

	local ok, definitionOrError = pcall(VehicleLoader.load, model)
	if not ok then
		model:Destroy()
		warn("[KemudiEngine] vehicle spawn rejected: " .. tostring(definitionOrError))
		return nil
	end
	local definition = definitionOrError

	local rt = Runtime.new({
		vehicle_mass = definition.vehicleMass,
		drive_layout = definition.driveLayout,
		studs_per_meter = definition.studsPerMeter,
		drivetrain_config = definition.drivetrainConfig,
		suspension_config = definition.suspensionConfig,
		simulation_config = definition.simulationConfig,
		static_parts = groundZero:getStaticColliderParts(),
	})
	rt.on_snapshot = function(snapshot)
		if rt.active then SnapshotEvent:FireClient(player, snapshot) end
	end
	rt.on_telemetry = function(telemetry)
		if rt.active then TelemetryEvent:FireClient(player, telemetry) end
	end
	rt.on_transmission_alert = function(reason, code, kind)
		if rt.active then TransmissionAlertEvent:FireClient(player, reason, code, kind) end
	end

	local spawnOk, spawnError = pcall(function()
		rt:spawn(player, definition.nodeDefs, definition.beamDefs, definition.rootPart, definition.renderParts, definition.wheels, definition.driverSeat)
		configureDriverPrompt(rt, player, template.Name)
	end)
	if not spawnOk then
		rt:despawn()
		warn("[KemudiEngine] vehicle runtime rejected: " .. tostring(spawnError))
		return nil
	end

	vehicles[player.UserId] = rt
	groundZero:registerVehicle(player, rt)
	configuratorSync:registerVehicle(player, definition, rt.world)
	print(string.format(
		"[KemudiEngine] vehicle spawned for %s from %s; derived mass %.2f",
		player.Name,
		template.Name,
		definition.vehicleMass
	))
	return rt
end

-- ============================================================
-- Input handling
-- ============================================================

ControlsEvent.OnServerEvent:Connect(function(player, controls)
	local rt = getPlayerVehicle(player)
	if not rt then return end
	if type(controls) ~= "table" then return end
	local seat = rt.driver_seat
	local humanoid = getPlayerHumanoid(player)
	if not seat or not humanoid or seat.Occupant ~= humanoid then
		rt:setControls({ brake = 1, handbrake = true })
		return
	end

	local safe = {
		steering = math.clamp(tonumber(controls.steering) or 0, -1, 1),
		throttle = math.clamp(tonumber(controls.throttle) or 0, 0, 1),
		brake = math.clamp(tonumber(controls.brake) or 0, 0, 1),
		clutch = math.clamp(tonumber(controls.clutch) or 0, 0, 1),
		handbrake = controls.handbrake == true,
		gear_up = controls.gear_up == true,
		gear_down = controls.gear_down == true,
		engine_on = rt.engine_on,
		selector_up = controls.selector_up == true,
		selector_down = controls.selector_down == true,
		manual_shift_up = controls.manual_shift_up == true,
		manual_shift_down = controls.manual_shift_down == true,
	}
	rt:setControls(safe)
end)

EngineToggleEvent.OnServerEvent:Connect(function(player)
	local rt = getPlayerVehicle(player)
	if not rt then return end
	local seat = rt.driver_seat
	local humanoid = getPlayerHumanoid(player)
	if not seat or not humanoid or seat.Occupant ~= humanoid then return end
	local state = rt:cycleEngineState()
	print(string.format("[KemudiEngine] engine state %s for %s", state, player.Name))
end)

-- ============================================================
-- Spawn/despawn
-- ============================================================

SpawnEvent.OnServerEvent:Connect(function(player, requestedId)
	spawnVehicle(player, requestedId)
end)

DespawnEvent.OnServerEvent:Connect(function(player)
	local rt = getPlayerVehicle(player)
	if rt then
		groundZero:unregisterVehicle(player)
		configuratorSync:unregisterVehicle(player)
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
		groundZero:unregisterVehicle(player)
		configuratorSync:unregisterVehicle(player)
		rt:despawn()
		vehicles[player.UserId] = nil
	end
end)

print("[KemudiEngine] Server runtime initialized")

return true
