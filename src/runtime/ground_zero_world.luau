--[[
	GroundZero procedural chunk streamer.

	This is intentionally server-owned. Roblox StreamingEnabled only streams
	existing Instances; this module creates and destroys a bounded set of
	deterministic tuning-surface chunks around active players/vehicles.

	Phase 1 provides an infinite-looking flat asphalt range. The custom vehicle
	solver already uses an analytic flat terrain fallback. Future test facilities
	can add explicit chunk templates and register their bounded physics colliders
	at the Runtime boundary.
--]]

local Players = game:GetService("Players")
local RunService = game:GetService("RunService")
local Workspace = game:GetService("Workspace")

local Grid = require(script.Parent.ground_zero_grid)

local GroundZeroWorld = {}
GroundZeroWorld.__index = GroundZeroWorld

local DEFAULTS = {
	chunkSize = 512,
	loadRadius = 2,
	unloadRadius = 3,
	generationBudget = 2,
	maxActiveChunks = 256,
	floorY = 0,
}

local function boundedInteger(value, fallback, minimum, maximum)
	if type(value) ~= "number" or value ~= value then
		return fallback
	end
	return math.clamp(math.floor(value), minimum, maximum)
end

local function validChunkSize(value)
	return type(value) == "number" and value == value and value >= 128 and value <= 2048
end

local function runtimePosition(runtime)
	if not runtime or not runtime.world then
		return nil
	end

	local nodes = runtime.world.nodes
	local x, y, z, count = 0, 0, 0, 0
	for _, node in nodes do
		if not node.fixed and node.x == node.x and node.y == node.y and node.z == node.z then
			x += node.x
			y += node.y
			z += node.z
			count += 1
		end
	end

	if count > 0 then
		local scale = if type(runtime.studs_per_meter) == "number" and runtime.studs_per_meter > 0 then runtime.studs_per_meter else 1
		return Vector3.new(x / count * scale, y / count * scale, z / count * scale)
	end

	if runtime.model then
		return runtime.model:GetPivot().Position
	end
	return nil
end

function GroundZeroWorld.new(config)
	config = config or {}
	local self = setmetatable({}, GroundZeroWorld)

	local chunkSize = if validChunkSize(config.chunkSize) then config.chunkSize else DEFAULTS.chunkSize
	self.chunkSize = chunkSize
	self.loadRadius = boundedInteger(config.loadRadius, DEFAULTS.loadRadius, 1, 8)
	self.unloadRadius = math.max(
		self.loadRadius + 1,
		boundedInteger(config.unloadRadius, DEFAULTS.unloadRadius, 2, 10)
	)
	self.generationBudget = boundedInteger(config.generationBudget, DEFAULTS.generationBudget, 1, 8)
	self.maxActiveChunks = boundedInteger(config.maxActiveChunks, DEFAULTS.maxActiveChunks, 25, 512)
	self.floorY = if type(config.floorY) == "number" and config.floorY == config.floorY then config.floorY else DEFAULTS.floorY
	self.activeChunks = {}
	self.vehicleRuntimes = {}
	self.connection = nil
	self.pendingCount = 0
	self.lastDiagnosticAt = 0
	self.lastFocusCellX = nil
	self.lastFocusCellZ = nil

	self.mapsFolder = Workspace:FindFirstChild("Maps")
	if not self.mapsFolder then
		self.mapsFolder = Instance.new("Folder")
		self.mapsFolder.Name = "Maps"
		self.mapsFolder.Parent = Workspace
	end

	self.groundZero = self.mapsFolder:FindFirstChild("GroundZero")
	if not self.groundZero then
		self.groundZero = Instance.new("Folder")
		self.groundZero.Name = "GroundZero"
		self.groundZero.Parent = self.mapsFolder
	end

	self.activeFolder = self.groundZero:FindFirstChild("ActiveChunks")
	if not self.activeFolder then
		self.activeFolder = Instance.new("Folder")
		self.activeFolder.Name = "ActiveChunks"
		self.activeFolder.Parent = self.groundZero
	end

	-- The authored Baseplate is finite and sits above the analytic solver ground.
	-- Hide it at runtime so generated GroundZero chunks are the visible/collidable
	-- surface. This is not saved back into the Studio-authored place.
	self.legacyBaseplate = self.groundZero:FindFirstChild("Baseplate", true)
	if self.legacyBaseplate and self.legacyBaseplate:IsA("BasePart") then
		self.legacyBaseplate.Transparency = 1
		self.legacyBaseplate.CanCollide = false
		self.legacyBaseplate.CanQuery = false
		self.legacyBaseplate.CanTouch = false
	end

	self.groundZero:SetAttribute("GroundZeroChunkSize", self.chunkSize)
	self.groundZero:SetAttribute("GroundZeroLoadRadius", self.loadRadius)
	self.groundZero:SetAttribute("GroundZeroUnloadRadius", self.unloadRadius)
	self.groundZero:SetAttribute("GroundZeroStatus", "Starting")

	self.spawnMarker = self.groundZero:FindFirstChild("CarSpawn", true)
	return self
end

function GroundZeroWorld:getSpawnCFrame()
	if self.spawnMarker and self.spawnMarker:IsA("BasePart") then
		return self.spawnMarker.CFrame * CFrame.new(0, 3, 0)
	end
	return CFrame.new(0, 3, 0)
end

-- Return authored, immovable map parts for the custom solver. Roblox's
-- CanCollide flag only affects Roblox physics; it is not consumed by the
-- node/beam world. Infrastructure parts are excluded because the solver has
-- its own analytic ground and spawn marker.
function GroundZeroWorld:getStaticColliderParts()
	local parts = {}
	for _, instance in self.groundZero:GetDescendants() do
		if not instance:IsA("BasePart") or not instance.CanCollide then
			continue
		end
		if instance == self.legacyBaseplate or instance == self.spawnMarker then
			continue
		end
		if self.activeFolder and instance:IsDescendantOf(self.activeFolder) then
			continue
		end
		if instance.Anchored then
			table.insert(parts, instance)
		else
			warn(string.format("[GroundZeroWorld] ignoring non-anchored static collider %s", instance:GetFullName()))
		end
	end
	return parts
end

function GroundZeroWorld:registerVehicle(player, runtime)
	if player and runtime then
		self.vehicleRuntimes[player.UserId] = runtime
	end
end

function GroundZeroWorld:unregisterVehicle(player)
	if player then
		self.vehicleRuntimes[player.UserId] = nil
	end
end

function GroundZeroWorld:_playerPositions()
	local positions = {}
	for _, player in Players:GetPlayers() do
		local runtime = self.vehicleRuntimes[player.UserId]
		local authoritativePosition = runtimePosition(runtime)
		if authoritativePosition then
			table.insert(positions, authoritativePosition)
			continue
		end

		local vehicle = Workspace:FindFirstChild("Vehicle_" .. player.Name)
		if vehicle and vehicle:IsA("Model") then
			table.insert(positions, vehicle:GetPivot().Position)
			continue
		end

		local character = player.Character
		local root = if character then character:FindFirstChild("HumanoidRootPart") else nil
		if root and root:IsA("BasePart") then
			table.insert(positions, root.Position)
		end
	end

	if #positions == 0 then
		table.insert(positions, Vector3.zero)
	end
	return positions
end

function GroundZeroWorld:_requiredCells(positions)
	local required = {}
	for _, position in positions do
		local cellX, cellZ = Grid.cellForPosition(position.X, position.Z, self.chunkSize)
		for _, cell in Grid.cellsAround(cellX, cellZ, self.loadRadius) do
			required[cell.key] = cell
		end
	end
	return required
end

function GroundZeroWorld:_nearestDistance(cellX, cellZ, positions)
	local nearest = math.huge
	for _, position in positions do
		local playerX, playerZ = Grid.cellForPosition(position.X, position.Z, self.chunkSize)
		local distance = math.max(math.abs(cellX - playerX), math.abs(cellZ - playerZ))
		if distance < nearest then
			nearest = distance
		end
	end
	return nearest
end

function GroundZeroWorld:_createFloorChunk(cellX, cellZ)
	local model = Instance.new("Model")
	model.Name = string.format("Chunk_%d_%d", cellX, cellZ)
	model.ModelStreamingMode = Enum.ModelStreamingMode.Atomic
	model:SetAttribute("GroundZeroChunkX", cellX)
	model:SetAttribute("GroundZeroChunkZ", cellZ)
	model:SetAttribute("GroundZeroChunkSize", self.chunkSize)
	model:SetAttribute("GroundZeroSurface", "Asphalt")

	local floor = Instance.new("Part")
	floor.Name = "AsphaltSurface"
	floor.Anchored = true
	floor.CanCollide = true
	floor.CanQuery = true
	floor.CanTouch = false
	floor.CastShadow = false
	floor.Material = Enum.Material.Asphalt
	floor.Color = Color3.fromRGB(42, 42, 45)
	floor.Size = Vector3.new(self.chunkSize, 1, self.chunkSize)
	local centerX, centerY, centerZ = Grid.center(cellX, cellZ, self.chunkSize, self.floorY - 0.5)
	floor.CFrame = CFrame.new(centerX, centerY, centerZ)
	floor.Parent = model

	model.PrimaryPart = floor
	model.Parent = self.activeFolder
	return model
end

function GroundZeroWorld:_createChunk(cell)
	if self.activeChunks[cell.key] then
		return
	end

	-- A future authored chunk template may replace this generated flat tile.
	-- Keep the first implementation deterministic and dependency-free.
	local model = self:_createFloorChunk(cell.x, cell.z)
	self.activeChunks[cell.key] = {
		model = model,
		x = cell.x,
		z = cell.z,
	}
end

function GroundZeroWorld:_destroyFarChunks(positions)
	for key, chunk in self.activeChunks do
		local nearby = false
		for _, position in positions do
			local cellX, cellZ = Grid.cellForPosition(position.X, position.Z, self.chunkSize)
			if Grid.isWithin(chunk.x, chunk.z, cellX, cellZ, self.unloadRadius) then
				nearby = true
				break
			end
		end

		if not nearby then
			if chunk.model then
				chunk.model:Destroy()
			end
			self.activeChunks[key] = nil
		end
	end
end

function GroundZeroWorld:_trimToLimit(positions)
	local chunks = {}
	for key, chunk in self.activeChunks do
		table.insert(chunks, {
			key = key,
			chunk = chunk,
			distance = self:_nearestDistance(chunk.x, chunk.z, positions),
		})
	end

	if #chunks <= self.maxActiveChunks then
		return
	end

	table.sort(chunks, function(a, b)
		return a.distance > b.distance
	end)
	for index = 1, #chunks - self.maxActiveChunks do
		local entry = chunks[index]
		if entry.chunk.model then
			entry.chunk.model:Destroy()
		end
		self.activeChunks[entry.key] = nil
	end
end

function GroundZeroWorld:update()
	local positions = self:_playerPositions()
	local focusCellX, focusCellZ = Grid.cellForPosition(positions[1].X, positions[1].Z, self.chunkSize)
	local required = self:_requiredCells(positions)
	local pending = {}

	for key, cell in required do
		if not self.activeChunks[key] then
			table.insert(pending, cell)
		end
	end

	table.sort(pending, function(a, b)
		return self:_nearestDistance(a.x, a.z, positions) < self:_nearestDistance(b.x, b.z, positions)
	end)

	local budget = math.min(self.generationBudget, #pending)
	for index = 1, budget do
		self:_createChunk(pending[index])
	end
	self.pendingCount = #pending - budget

	self:_destroyFarChunks(positions)
	self:_trimToLimit(positions)

	local activeCount = 0
	for _ in self.activeChunks do
		activeCount += 1
	end
	self.groundZero:SetAttribute("GroundZeroActiveChunks", activeCount)
	self.groundZero:SetAttribute("GroundZeroPendingChunks", self.pendingCount)
	self.groundZero:SetAttribute("GroundZeroFocusCellX", focusCellX)
	self.groundZero:SetAttribute("GroundZeroFocusCellZ", focusCellZ)
	self.groundZero:SetAttribute("GroundZeroStatus", "Running")
	if activeCount > self.maxActiveChunks then
		warn(string.format(
			"[GroundZeroWorld] active chunk limit exceeded (%d/%d); reduce player spread or load radius",
			activeCount,
			self.maxActiveChunks
		))
	end

	local now = os.clock()
	if now - self.lastDiagnosticAt >= 5 then
		self.lastDiagnosticAt = now
		print(string.format(
			"[GroundZeroWorld] active=%d pending=%d focus=(%d,%d) loadRadius=%d chunkSize=%d",
			activeCount,
			self.pendingCount,
			focusCellX,
			focusCellZ,
			self.loadRadius,
			self.chunkSize
		))
	end
end

function GroundZeroWorld:disableForGeneratedMap()
	self:stop()
	for _, instance in self.groundZero:GetDescendants() do
		if instance:IsA("BasePart") then
			instance.Transparency = 1
			instance.CanCollide = false
			instance.CanQuery = false
			instance.CanTouch = false
		elseif instance:IsA("Decal") or instance:IsA("Texture") then
			instance.Transparency = 1
		elseif instance:IsA("ParticleEmitter") or instance:IsA("Trail") or instance:IsA("Beam") then
			instance.Enabled = false
		end
	end
	self.groundZero:SetAttribute("GroundZeroStatus", "DisabledForGeneratedMap")
end

function GroundZeroWorld:start()
	if self.connection then
		return
	end
	self:update()
	self.connection = RunService.Heartbeat:Connect(function()
		self:update()
	end)
	print("[GroundZeroWorld] streaming started")
end

function GroundZeroWorld:stop()
	if self.connection then
		self.connection:Disconnect()
		self.connection = nil
	end
	for key, chunk in self.activeChunks do
		if chunk.model then
			chunk.model:Destroy()
		end
		self.activeChunks[key] = nil
	end
	self.pendingCount = 0
	self.vehicleRuntimes = {}
	self.groundZero:SetAttribute("GroundZeroActiveChunks", 0)
	self.groundZero:SetAttribute("GroundZeroPendingChunks", 0)
	self.groundZero:SetAttribute("GroundZeroStatus", "Stopped")
end

return GroundZeroWorld
