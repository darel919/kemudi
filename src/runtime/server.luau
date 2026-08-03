--[[
	Server script — manages vehicle lifecycle, receives client input, sends snapshots.
	Goes in ServerScriptService.

	Vehicle geometry, node layout, proxy mass, wheels, and beams are authored in
	ServerStorage.KemudiVehicles. The client may request an asset ID, but cannot
	supply mass, tuning, topology, or physics configuration.
--]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local ServerStorage = game:GetService("ServerStorage")
local Workspace = game:GetService("Workspace")
local Players = game:GetService("Players")
local Runtime = require(ReplicatedStorage.KemudiEngine.runtime)
local VehicleLoader = require(ReplicatedStorage.KemudiEngine.runtime.vehicle_asset_loader)
local GroundZeroWorld = require(ReplicatedStorage.KemudiEngine.runtime.ground_zero_world)
local Configurator = require(ReplicatedStorage.KemudiEngine.configurator)

local mapFolder = ServerStorage:FindFirstChild("KemudiMaps")
local availableMaps = {}
local mapList = {}
if mapFolder then
	for _, candidate in ipairs(mapFolder:GetChildren()) do
		local module = candidate:FindFirstChild("Map")
		if module and module:IsA("ModuleScript") then
			local ok, loaded = pcall(require, module)
			local heightmap = if ok and type(loaded) == "table" then loaded.heightmap else nil
			local spawn = if ok and type(loaded) == "table" then loaded.spawn else nil
			if ok
				and type(loaded) == "table"
				and type(loaded.name) == "string"
				and #loaded.name > 0
				and type(heightmap) == "table"
				and type(heightmap.heights) == "table"
				and type(heightmap.samplesX) == "number"
				and type(heightmap.samplesZ) == "number"
				and type(heightmap.widthMeters) == "number"
				and heightmap.widthMeters > 0
				and type(heightmap.depthMeters) == "number"
				and heightmap.depthMeters > 0
				and type(spawn) == "table"
				and type(spawn.x) == "number"
				and type(spawn.y) == "number"
				and type(spawn.z) == "number"
				and type(spawn.headingRadians) == "number"
				and #heightmap.heights == heightmap.samplesX * heightmap.samplesZ
			then
				local mapName = candidate.Name
				availableMaps[mapName] = loaded
				table.insert(mapList, {
					name = mapName,
					widthMeters = heightmap.widthMeters,
					depthMeters = heightmap.depthMeters,
					samplesX = heightmap.samplesX,
					samplesZ = heightmap.samplesZ,
				})
				print(string.format("[KemudiEngine] map loaded: %s", mapName))
			else
				warn(string.format("[KemudiEngine] map rejected: %s", tostring(loaded)))
			end
		end
	end
end
table.sort(mapList, function(a, b)
	return a.name < b.name
end)
local defaultMapName = if mapList[1] then mapList[1].name else nil
if not defaultMapName then
	print("[KemudiEngine] no generated maps installed; using GroundZero as active world")
end

local function hasVisibleMapGeometry(mapName)
	local mapsInWorkspace = Workspace:FindFirstChild("Maps")
	local mapContainer = if mapsInWorkspace then mapsInWorkspace:FindFirstChild(mapName) else nil
	if not mapContainer then return false end
	for _, instance in mapContainer:GetDescendants() do
		if instance:IsA("BasePart") and instance.Transparency < 1 and instance.Size.Magnitude > 0 then
			return true
		end
	end
	return false
end

local function fixPartAppearance(part: BasePart)
	part.Transparency = 0
	part.Material = Enum.Material.Slate
	part.CastShadow = true
end

local function fixModelAppearance(model: Model)
	for _, descendant in model:GetDescendants() do
		if descendant:IsA("BasePart") then
			fixPartAppearance(descendant)
		elseif descendant:IsA("Decal") or descendant:IsA("Texture") then
			descendant.Transparency = 0
		end
	end
end

local function scaleMapGeometryToRoblox(mapName, studsPerMeter)
	local mapsInWorkspace = Workspace:FindFirstChild("Maps")
	local mapContainer = if mapsInWorkspace then mapsInWorkspace:FindFirstChild(mapName) else nil
	if not mapContainer then return false end

	local scaled = false
	for _, instance in mapContainer:GetChildren() do
		if instance:IsA("Model") then
			local pivot = instance:GetPivot()
			local rotation = pivot - pivot.Position
			instance:ScaleTo(studsPerMeter)
			instance:PivotTo(CFrame.new(pivot.Position * studsPerMeter) * rotation)
			fixModelAppearance(instance)
			scaled = true
		elseif instance:IsA("BasePart") then
			local position = instance.Position
			local rotation = instance.CFrame - position
			instance.Size *= studsPerMeter
			instance.CFrame = CFrame.new(position * studsPerMeter) * rotation
			fixPartAppearance(instance)
			scaled = true
		end
	end

	if scaled then
		mapContainer:SetAttribute("KemudiVisualScale", studsPerMeter)
		print(string.format(
			"[KemudiEngine] scaled %s visual geometry by %.4f studs per meter",
			mapName,
			studsPerMeter
		))
	end
	return scaled
end

local groundZero = GroundZeroWorld.new({
	chunkSize = 512,
	loadRadius = 2,
	unloadRadius = 3,
	generationBudget = 2,
	maxActiveChunks = 256,
})
local visibleMapGeometry = if defaultMapName then hasVisibleMapGeometry(defaultMapName) else false
if defaultMapName and visibleMapGeometry then
	groundZero:disableForGeneratedMap()
	print(string.format("[KemudiEngine] generated map mode active: %s; GroundZero visuals disabled", defaultMapName))
else
	if defaultMapName then
		warn(string.format(
			"[KemudiEngine] no visible geometry found at Workspace.Maps.%s; keeping GroundZero fallback",
			defaultMapName
		))
	end
	groundZero:start()
end

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
local MapListFunction = getOrCreate("KemudiMapList", "RemoteFunction")
MapListFunction.OnServerInvoke = function(player)
	-- Return a fresh shallow copy so the client cannot mutate server registry data.
	local result = table.create(#mapList)
	for index, mapInfo in ipairs(mapList) do
		result[index] = {
			name = mapInfo.name,
			widthMeters = mapInfo.widthMeters,
			depthMeters = mapInfo.depthMeters,
			samplesX = mapInfo.samplesX,
			samplesZ = mapInfo.samplesZ,
		}
	end
	return result
end

-- ============================================================
-- Vehicle registry
-- ============================================================

local vehicles = {} -- playerUserId → Runtime instance
local characterSpawnCFrames = {} -- playerUserId → map spawn CFrame
local characterSpawnConnections = {} -- playerUserId → CharacterAdded connection
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

local function getMapSpawnCFrame(mapData, studsPerMeter)
	local spawn = mapData.spawn
	return CFrame.new(
		spawn.x * studsPerMeter,
		spawn.y * studsPerMeter,
		spawn.z * studsPerMeter
	) * CFrame.Angles(0, spawn.headingRadians, 0)
end

local function moveCharacterToMapSpawn(player, character)
	local spawnCFrame = characterSpawnCFrames[player.UserId]
	if not spawnCFrame or not character or not character.Parent then return end
	local humanoid = character:FindFirstChildOfClass("Humanoid")
	if humanoid then humanoid.Sit = false end
	character:PivotTo(spawnCFrame * CFrame.new(0, 5, 0))
	local position = character:GetPivot().Position
	print(string.format(
		"[KemudiEngine] character %s moved to selected map spawn at (%.2f, %.2f, %.2f) studs",
		player.Name,
		position.X,
		position.Y,
		position.Z
	))
end

local function setCharacterMapSpawn(player, spawnCFrame)
	characterSpawnCFrames[player.UserId] = spawnCFrame
	if not characterSpawnConnections[player.UserId] then
		characterSpawnConnections[player.UserId] = player.CharacterAdded:Connect(function(character)
			task.defer(moveCharacterToMapSpawn, player, character)
		end)
	end
	if player.Character then
		task.defer(moveCharacterToMapSpawn, player, player.Character)
	end
end

local function clearCharacterMapSpawn(player)
	characterSpawnCFrames[player.UserId] = nil
	local connection = characterSpawnConnections[player.UserId]
	if connection then connection:Disconnect() end
	characterSpawnConnections[player.UserId] = nil
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

local function resolveMap(requestedMapName)
	if type(requestedMapName) == "string" and #requestedMapName > 0 and #requestedMapName <= 64 then
		local requested = availableMaps[requestedMapName]
		if requested then
			return requestedMapName, requested
		end
	end
	if defaultMapName then
		return defaultMapName, availableMaps[defaultMapName]
	end
	return nil, nil
end

-- ============================================================
-- Spawn a vehicle for a player
-- ============================================================

local function spawnVehicle(player, requestedId, requestedMapName)
	local mapName, mapData = resolveMap(requestedMapName)
	if not mapData then
		mapName = "GroundZero"
		mapData = nil
		print("[KemudiEngine] no generated map selected; spawning at GroundZero")
	end
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

	local ok, definitionOrError = pcall(VehicleLoader.load, model)
	if not ok then
		model:Destroy()
		warn("[KemudiEngine] vehicle spawn rejected: " .. tostring(definitionOrError))
		return nil
	end
	local definition = definitionOrError

	local spawnCFrame
	if mapData then
		scaleMapGeometryToRoblox(mapName, definition.studsPerMeter)
		spawnCFrame = getMapSpawnCFrame(mapData, definition.studsPerMeter)
	else
		spawnCFrame = groundZero:getSpawnCFrame()
	end
	local authoredPivot = model:GetPivot()
	local authoredRotation = authoredPivot - authoredPivot.Position
	model:PivotTo(spawnCFrame * authoredRotation)

	local rt = Runtime.new({
		vehicle_mass = definition.vehicleMass,
		drive_layout = definition.driveLayout,
		studs_per_meter = definition.studsPerMeter,
		drivetrain_config = definition.drivetrainConfig,
		suspension_config = definition.suspensionConfig,
		simulation_config = definition.simulationConfig,
		map_data = mapData,
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
	setCharacterMapSpawn(player, spawnCFrame)
	groundZero:registerVehicle(player, rt)
	configuratorSync:registerVehicle(player, definition, rt.world)
	print(string.format(
		"[KemudiEngine] vehicle spawned for %s from %s on map %s; derived mass %.2f",
		player.Name,
		template.Name,
		mapName,
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

SpawnEvent.OnServerEvent:Connect(function(player, requestedId, requestedMapName)
	spawnVehicle(player, requestedId, requestedMapName)
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
	clearCharacterMapSpawn(player)
end)

print("[KemudiEngine] Server runtime initialized")

return true
