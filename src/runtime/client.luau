--[[
	Client script — reads player input, sends to server, receives snapshots.
	Goes in StarterPlayerScripts.
--]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local UserInputService = game:GetService("UserInputService")
local RunService = game:GetService("RunService")
local Players = game:GetService("Players")
local SoundService = game:GetService("SoundService")

-- Remote events
local ControlsEvent = ReplicatedStorage:WaitForChild("KemudiControls")
local SnapshotEvent = ReplicatedStorage:WaitForChild("KemudiSnapshot")
local TelemetryEvent = ReplicatedStorage:WaitForChild("KemudiTelemetry")
local SpawnEvent = ReplicatedStorage:WaitForChild("KemudiSpawn")
local DespawnEvent = ReplicatedStorage:WaitForChild("KemudiDespawn")
local MapListFunction = ReplicatedStorage:WaitForChild("KemudiMapList")
local EngineToggleEvent = ReplicatedStorage:WaitForChild("KemudiEngineToggle")
local TransmissionAlertEvent = ReplicatedStorage:WaitForChild("KemudiTransmissionAlert")
local Engine = ReplicatedStorage:WaitForChild("KemudiEngine")
local Chassis = require(Engine:WaitForChild("runtime"):WaitForChild("chassis"))
local Contract = require(Engine:WaitForChild("contract"))
local ClientInput = require(Engine:WaitForChild("runtime"):WaitForChild("client_input"))
local ClientHud = require(Engine:WaitForChild("runtime"):WaitForChild("client_hud"))

local local_player = Players.LocalPlayer

-- Initialize input module
ClientInput.init({
	localPlayer = local_player,
	EngineToggleEvent = EngineToggleEvent,
	UserInputService = UserInputService,
})

-- Initialize HUD module
local hud = ClientHud.init({
	localPlayer = local_player,
	SoundService = SoundService,
	Contract = Contract,
	playerGui = (local_player or Players.LocalPlayer):WaitForChild("PlayerGui"),
	TransmissionAlertEvent = TransmissionAlertEvent,
})

-- Spawn request
-- ============================================================

local selected_map_name = nil

local function requestSpawn(mapName)
	-- The server resolves the authored vehicle asset and validates the map name.
	-- No mass, tuning, or heightmap data crosses the trust boundary from client.
	SpawnEvent:FireServer(nil, mapName)
end

local function requestDespawn()
	DespawnEvent:FireServer()
end

-- Snapshot handling
-- ============================================================

local node_parts = {} -- map: nodeIndex → Part (client-side presentation)
local latest_telemetry = nil
local NODE_NAMES = { "Node_FL", "Node_FR", "Node_RL", "Node_RR" }

local node_parts = {}
local NODE_NAMES = { "Node_FL", "Node_FR", "Node_RL", "Node_RR" }
local hidden_character = nil
local saved_local_transparency = {}

local function setAvatarHidden(hidden)
	local character = local_player.Character
	if hidden then
		if not character then return end
		if hidden_character ~= character then
			hidden_character = character
			saved_local_transparency = {}
		end
		for _, descendant in character:GetDescendants() do
			if descendant:IsA("BasePart") then
				if saved_local_transparency[descendant] == nil then
					saved_local_transparency[descendant] = descendant.LocalTransparencyModifier
				end
				descendant.LocalTransparencyModifier = 1
			end
		end
	else
		for part, transparency in saved_local_transparency do
			if part.Parent then
				part.LocalTransparencyModifier = transparency
			end
		end
		saved_local_transparency = {}
		hidden_character = nil
	end
end


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
	local model = workspace:FindFirstChild("Vehicle_" .. local_player.Name)
	local studs_per_meter = 1
	if model then
		local spec = model:FindFirstChild("VehicleSpec")
		local authoredScale = if spec then spec:GetAttribute("StudsPerMeter") else nil
		if type(authoredScale) == "number" and authoredScale > 0 then
			studs_per_meter = authoredScale
		end
	end
	if next(node_parts) == nil then refreshNodeParts() end
	local snapshot_nodes = table.create(node_count)

	for i = 0, node_count - 1 do
		local base = i * 6 + 2
		local x = snapshot[base] * studs_per_meter
		local y = snapshot[base + 1] * studs_per_meter
		local z = snapshot[base + 2] * studs_per_meter
		snapshot_nodes[i + 1] = { x = x, y = y, z = z }
	end

	local chassisFrame = Chassis.frameFromNodes(snapshot_nodes)
	for i = 0, node_count - 1 do
		local base = i * 6 + 2
		local x = snapshot[base] * studs_per_meter
		local y = snapshot[base + 1] * studs_per_meter
		local z = snapshot[base + 2] * studs_per_meter

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
	if type(telemetry) == "table" then
		latest_telemetry = telemetry
		ClientHud.updateHud(telemetry)
	end
end)
-- ============================================================
-- Main loop
-- ============================================================

local send_interval = 1 / 20 -- 20 Hz input rate
local send_timer = 0

RunService.Heartbeat:Connect(function(dt)
	ClientInput.processInput(dt, UserInputService)
	local driving = ClientInput.isDriving()
	if driving ~= last_driving_state or (driving and hidden_character ~= local_player.Character) then
		setAvatarHidden(driving)
		hud.hud_gui.Enabled = driving
		if not driving then
			hud.incrementWarningToken()
			hud.transmission_warning_label.Visible = false
			hud.resetSpec()
		end
		last_driving_state = driving
	end

	send_timer += dt
	if send_timer >= send_interval then
		send_timer = 0
		if ClientInput.isDriving() then
			ControlsEvent:FireServer(ClientInput.input)
			-- These are commands, not held analog values. Clear only after the
			-- command has crossed the network boundary.
			ClientInput.input.gear_up = false
			ClientInput.input.gear_down = false
			ClientInput.input.manual_shift_up = false
			ClientInput.input.manual_shift_down = false
			ClientInput.input.selector_up = false
			ClientInput.input.selector_down = false
		end
	end
end)
-- MainMenu (map selection → spawn)
local MainMenu = require(Engine:WaitForChild("runtime"):WaitForChild("main_menu"))
MainMenu.init({
	playerGui = local_player:WaitForChild("PlayerGui"),
	mapListFunction = MapListFunction,
	onStart = function(mapName)
		selected_map_name = mapName
		requestSpawn(mapName)
	end,
})

-- Configurator UI (real-time vehicle tuning)
-- ============================================================

local ConfiguratorUI = require(ReplicatedStorage:WaitForChild("KemudiEngine"):WaitForChild("configurator"):WaitForChild("ui"))
ConfiguratorUI.init()

-- Dashboard toggle buttons
-- Configurator toggle
local cfg_toggle = Instance.new("TextButton")
cfg_toggle.Name = "ConfigToggle"
cfg_toggle.Size = UDim2.fromOffset(44, 20)
cfg_toggle.Position = UDim2.fromOffset(128, 6)
cfg_toggle.BackgroundColor3 = Color3.fromRGB(18, 30, 42)
cfg_toggle.Text = "CFG"
cfg_toggle.TextColor3 = Color3.fromRGB(175, 210, 225)
cfg_toggle.Font = Enum.Font.Code
cfg_toggle.TextSize = 10
cfg_toggle.BorderSizePixel = 0
	cfg_toggle.Parent = hud.dashboard
local cfg_corner = Instance.new("UICorner")
cfg_corner.CornerRadius = UDim.new(0, 4)
cfg_corner.Parent = cfg_toggle
cfg_toggle.MouseButton1Click:Connect(function()
	ConfiguratorUI.toggle()
end)

-- OBD toggle
local obd_toggle = Instance.new("TextButton")
obd_toggle.Name = "OBDToggle"
obd_toggle.Size = UDim2.fromOffset(44, 20)
obd_toggle.Position = UDim2.fromOffset(174, 6)
obd_toggle.BackgroundColor3 = Color3.fromRGB(18, 30, 42)
obd_toggle.Text = "OBD"
obd_toggle.TextColor3 = Color3.fromRGB(175, 210, 225)
obd_toggle.Font = Enum.Font.Code
obd_toggle.TextSize = 10
obd_toggle.BorderSizePixel = 0
	obd_toggle.Parent = hud.dashboard
local obd_corner = Instance.new("UICorner")
obd_corner.CornerRadius = UDim.new(0, 4)
obd_corner.Parent = obd_toggle
obd_toggle.MouseButton1Click:Connect(function()
	hud.obd_panel.Visible = not hud.obd_panel.Visible
end)

-- Vehicle spawn is started by the map-selection menu. The server validates the
-- requested map name and falls back to its default when necessary.

print("[KemudiEngine] Client runtime initialized")

return true