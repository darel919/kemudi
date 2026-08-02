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
local EngineToggleEvent = ReplicatedStorage:WaitForChild("KemudiEngineToggle")
local Engine = ReplicatedStorage:WaitForChild("KemudiEngine")
local Chassis = require(Engine:WaitForChild("runtime"):WaitForChild("chassis"))
local Contract = require(Engine:WaitForChild("contract"))

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
}

-- ============================================================
-- Input processing
-- ============================================================

local keys_held = {}

local function isDriving()
	local character = Players.LocalPlayer.Character
	local humanoid = if character then character:FindFirstChildOfClass("Humanoid") else nil
	local seat = if humanoid then humanoid.SeatPart else nil
	return seat ~= nil and seat:GetAttribute("KemudiDriverSeat") == true
end

UserInputService.InputBegan:Connect(function(gameInput, processed)
	if processed then return end
	if gameInput.UserInputType == Enum.UserInputType.Keyboard then
		if gameInput.KeyCode == Enum.KeyCode.F and isDriving() then
			EngineToggleEvent:FireServer()
			return
		end
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
	-- Edge-triggered gear shifts
	if isDriving() and keys_held[Enum.KeyCode.E] then
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
	-- The server resolves the authored default asset. No mass or tuning crosses
	-- the trust boundary from the client.
	SpawnEvent:FireServer()
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
local NODE_NAMES = { "Node_FL", "Node_FR", "Node_RL", "Node_RR" }

-- ============================================================
-- HUD / ECU / OBD presentation
-- ============================================================

local player_gui = local_player:WaitForChild("PlayerGui")
local hud_gui = player_gui:FindFirstChild("KemudiHUD")
if not hud_gui then
	local created = Instance.new("ScreenGui")
	created.Name = "KemudiHUD"
	created.ResetOnSpawn = false
	created.IgnoreGuiInset = true
	created.DisplayOrder = 20
	created.Parent = player_gui
	hud_gui = created
end
hud_gui.Enabled = true

local function makePanel(parent, name, size, position, anchorPoint)
	local panel = Instance.new("Frame")
	panel.Name = name
	panel.Size = size
	panel.Position = position
	panel.AnchorPoint = anchorPoint or Vector2.zero
	panel.BackgroundColor3 = Color3.fromRGB(10, 16, 24)
	panel.BackgroundTransparency = 0.18
	panel.BorderSizePixel = 0
	panel.Parent = parent
	local stroke = Instance.new("UIStroke")
	stroke.Color = Color3.fromRGB(80, 170, 220)
	stroke.Transparency = 0.35
	stroke.Parent = panel
	return panel
end

local function makeLabel(parent, name, position, size, text, textSize)
	local label = Instance.new("TextLabel")
	label.Name = name
	label.Position = position
	label.Size = size
	label.BackgroundTransparency = 1
	label.TextColor3 = Color3.fromRGB(235, 245, 255)
	label.Font = Enum.Font.Code
	label.TextSize = textSize or 14
	label.TextXAlignment = Enum.TextXAlignment.Left
	label.TextYAlignment = Enum.TextYAlignment.Top
	label.Text = text
	label.Parent = parent
	return label
end

local dashboard = makePanel(
	hud_gui,
	"Dashboard",
	UDim2.fromOffset(300, 158),
	UDim2.new(1, -22, 1, -22),
	Vector2.new(1, 1)
)
makeLabel(dashboard, "Title", UDim2.fromOffset(12, 8), UDim2.fromOffset(270, 20), "KEMUDI // DRIVER HUD", 15)
local speed_label = makeLabel(dashboard, "Speed", UDim2.fromOffset(12, 33), UDim2.fromOffset(170, 42), "000 km/h", 30)
local gear_label = makeLabel(dashboard, "Gear", UDim2.fromOffset(190, 36), UDim2.fromOffset(95, 30), "GEAR N", 18)
local status_label = makeLabel(dashboard, "Status", UDim2.fromOffset(12, 78), UDim2.fromOffset(270, 22), "ENGINE OFF // PRESS F", 14)
local input_label = makeLabel(dashboard, "Inputs", UDim2.fromOffset(12, 105), UDim2.fromOffset(270, 42), "THR 0%  BRK 0%  STR 0%", 13)

local ecu_panel = makePanel(
	hud_gui,
	"ECU",
	UDim2.fromOffset(290, 178),
	UDim2.fromOffset(22, 22)
)
makeLabel(ecu_panel, "Title", UDim2.fromOffset(12, 8), UDim2.fromOffset(260, 20), "ECU // LIVE DATA", 15)
local ecu_label = makeLabel(ecu_panel, "Values", UDim2.fromOffset(12, 34), UDim2.fromOffset(265, 135), "RPM 0\nTHR 0%\nBRK 0%\nSTEER 0%\nCLUTCH 0%\nTRANS 20 C", 13)

local obd_panel = makePanel(
	hud_gui,
	"OBD",
	UDim2.fromOffset(290, 108),
	UDim2.new(0, 22, 1, -22),
	Vector2.new(0, 1)
)
makeLabel(obd_panel, "Title", UDim2.fromOffset(12, 8), UDim2.fromOffset(260, 20), "OBD-II // DIAGNOSTICS", 15)
local obd_label = makeLabel(obd_panel, "Values", UDim2.fromOffset(12, 34), UDim2.fromOffset(265, 65), "LINK CONNECTED\nNO ACTIVE DTC", 13)

local function telemetryValue(data, index)
	local value = data[index + 1]
	return if type(value) == "number" and value == value then value else 0
end

local function gearText(value)
	local gear = math.floor(value + 0.5)
	if gear < 0 then return "R" end
	if gear == 0 then return "N" end
	return tostring(gear)
end

local function updateHud(data)
	local speed = telemetryValue(data, Contract.T.SPEED_KMH)
	local rpm = telemetryValue(data, Contract.T.RPM)
	local gear = telemetryValue(data, Contract.T.GEAR)
	local throttle = telemetryValue(data, Contract.T.THROTTLE) * 100
	local brake = telemetryValue(data, Contract.T.BRAKE) * 100
	local steering = telemetryValue(data, Contract.T.STEERING) * 100
	local clutch = telemetryValue(data, Contract.T.CLUTCH) * 100
	local transTemp = telemetryValue(data, Contract.T.TRANS_TEMP)
	local engineStage = math.floor(telemetryValue(data, Contract.T.ENGINE_STAGE) + 0.5)
	local warning = telemetryValue(data, Contract.T.ENGINE_WARNING)
	local tcmFault = telemetryValue(data, Contract.T.TCM_FAULT_MASK)
	local diagnosticCode = telemetryValue(data, Contract.T.TCM_DIAGNOSTIC_CODE)

	speed_label.Text = string.format("%03d km/h", math.max(0, math.floor(speed + 0.5)))
	gear_label.Text = "GEAR " .. gearText(gear)
	local engineText = if engineStage == 1 then "ACCESSORY" elseif engineStage == 2 then "ENGINE ON" else "ENGINE OFF"
	status_label.Text = if isDriving() then engineText .. (if engineStage == 2 then " // DRIVE READY" else " // PRESS F") else "EXITED // PRESS ENTER"
	input_label.Text = string.format("THR %02d%%  BRK %02d%%  STR %+03d%%", throttle, brake, steering)
	ecu_label.Text = string.format(
		"RPM %04d\nTHR %02d%%\nBRK %02d%%\nSTEER %+03d%%\nCLUTCH %02d%%\nTRANS %.1f C",
		math.max(0, math.floor(rpm + 0.5)),
		math.clamp(math.floor(throttle + 0.5), 0, 100),
		math.clamp(math.floor(brake + 0.5), 0, 100),
		math.floor(steering + 0.5),
		math.clamp(math.floor(clutch + 0.5), 0, 100),
		transTemp
	)
	local diagnostic = "NO ACTIVE DTC"
	if warning > 0 then
		diagnostic = "ENGINE WARNING"
	elseif tcmFault > 0 then
		diagnostic = string.format("TCM FAULT %d", math.floor(tcmFault))
	elseif diagnosticCode > 0 then
		diagnostic = string.format("TCM DTC %d", math.floor(diagnosticCode))
	end
	obd_label.Text = "LINK CONNECTED\n" .. diagnostic
end

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

local last_driving_state = false

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
	-- Consumers can subscribe to this adapter later; retaining the latest packet
	-- keeps the replication boundary observable without logging every frame.
	if type(telemetry) == "table" then
		latest_telemetry = telemetry
		updateHud(telemetry)
	end
end)

-- ============================================================
-- Main loop
-- ============================================================

local send_interval = 1 / 20 -- 20 Hz input rate
local send_timer = 0

RunService.Heartbeat:Connect(function(dt)
	processInput(dt)
	local driving = isDriving()
	if driving ~= last_driving_state or (driving and hidden_character ~= local_player.Character) then
		setAvatarHidden(driving)
		last_driving_state = driving
	end

	send_timer += dt
	if send_timer >= send_interval then
		send_timer = 0
		if isDriving() then
			ControlsEvent:FireServer(input)
		end
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

print("[KemudiEngine] Client runtime initialized")

return true
