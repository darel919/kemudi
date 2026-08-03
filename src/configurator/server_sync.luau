--[[
	Server-side configurator sync — validates client requests against the
	manifest, applies changes to the authoritative World, and broadcasts
	config state to all clients.

	Sits in ServerScriptService alongside server.luau.
	Pure Luau logic; creates RemoteEvents for network transport.
--]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local Manifest = require(ReplicatedStorage.KemudiEngine.configurator.manifest)
local Configurator = require(ReplicatedStorage.KemudiEngine.configurator.vehicle_configurator)

local ServerSync = {}
ServerSync.__index = ServerSync

-- ============================================================
-- Remote event setup
-- ============================================================

local function getOrCreate(name, className)
	local obj = ReplicatedStorage:FindFirstChild(name)
	if obj then return obj end
	obj = Instance.new(className)
	obj.Name = name
	obj.Parent = ReplicatedStorage
	return obj
end

local ConfigChangeEvent = getOrCreate("KemudiConfigChange", "RemoteEvent")
local ConfigSyncEvent = getOrCreate("KemudiConfigSync", "RemoteEvent")
local ConfigManifestEvent = getOrCreate("KemudiConfigManifest", "RemoteEvent")

-- ============================================================
-- Manager
-- ============================================================

--- @param vehicles table  — map of userId → Runtime instance (shared with server.luau)
function ServerSync.new(vehicles)
	local self = setmetatable({}, ServerSync)
	self.vehicles = vehicles
	self.configurators = {} -- userId → Configurator instance
	self.manifests = {}    -- userId → manifest

	-- Handle client config change requests
	ConfigChangeEvent.OnServerEvent:Connect(function(player, request)
		self:handleRequest(player, request)
	end)

	return self
end

--- Called after a vehicle is spawned for a player.
--- Builds the manifest and creates a configurator instance.
--- Sends the manifest to the client.
--- @param player Player
--- @param definition table  — vehicle asset loader output
--- @param world table  — the physics World instance
function ServerSync:registerVehicle(player, definition, world)
	-- Build capability manifest
	local manifest = Manifest.build(definition)
	self.manifests[player.UserId] = manifest

	-- Create configurator
	local configurator = Configurator.new(manifest, world)
	self.configurators[player.UserId] = configurator

	-- Send manifest to the owning client
	ConfigManifestEvent:FireClient(player, manifest)
end

--- Called when a vehicle is despawned.
function ServerSync:unregisterVehicle(player)
	self.configurators[player.UserId] = nil
	self.manifests[player.UserId] = nil
end

-- ============================================================
-- Request handling
-- ============================================================

function ServerSync:handleRequest(player, request)
	if type(request) ~= "table" then return end

	local rt = self.vehicles[player.UserId]
	if not rt then return end

	-- Must be the driver
	local seat = rt.driver_seat
	if not seat then return end
	local humanoid = player.Character and player.Character:FindFirstChildOfClass("Humanoid")
	if not humanoid or seat.Occupant ~= humanoid then return end

	local configurator = self.configurators[player.UserId]
	if not configurator then return end

	local action = request.action
	if action == "change" then
		-- Single param change
		local category = request.category
		local param = request.param
		local value = request.value
		local wheelId = request.wheelId

		local ok, err = configurator:apply(category, param, value, wheelId)
		if not ok then
			-- Send error back to requesting client only
			ConfigSyncEvent:FireClient(player, {
				action = "error",
				category = category,
				param = param,
				error = err,
			})
			return
		end

		-- Broadcast the change to ALL clients in the session
		ConfigSyncEvent:FireAllClients({
			action = "changed",
			category = category,
			param = param,
			wheelId = wheelId,
			value = value,
		})

	elseif action == "reset_all" then
		configurator:resetAll()
		-- Broadcast full snapshot
		local snapshot = configurator:snapshot()
		ConfigSyncEvent:FireAllClients({
			action = "full_sync",
			snapshot = snapshot,
		})

	elseif action == "reset_category" then
		configurator:resetCategory(request.category)
		-- Broadcast full snapshot for the category
		local snapshot = configurator:snapshot()
		ConfigSyncEvent:FireAllClients({
			action = "full_sync",
			snapshot = snapshot,
		})

	elseif action == "request_snapshot" then
		-- Client requests current config state (e.g. after joining)
		local snapshot = configurator:snapshot()
		ConfigSyncEvent:FireClient(player, {
			action = "full_sync",
			snapshot = snapshot,
		})
	end
end

--- Broadcast the full config snapshot to a player (e.g. on join).
function ServerSync:sendFullSync(player)
	local configurator = self.configurators[player.UserId]
	if not configurator then return end
	local snapshot = configurator:snapshot()
	ConfigSyncEvent:FireClient(player, {
		action = "full_sync",
		snapshot = snapshot,
	})
end

return ServerSync
