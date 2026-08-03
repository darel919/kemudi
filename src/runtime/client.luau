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
local EngineToggleEvent = ReplicatedStorage:WaitForChild("KemudiEngineToggle")
local TransmissionAlertEvent = ReplicatedStorage:WaitForChild("KemudiTransmissionAlert")
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
	selector_up = false,
	selector_down = false,
	manual_shift_up = false,
	manual_shift_down = false,
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

-- Gamepad state (continuous analog values + edge-triggered buttons)
local gamepad_state = {
	steer = 0,        -- Left stick X: -1 to 1
	throttle = 0,     -- RT (ButtonR2): 0 to 1
	brake = 0,        -- LT (ButtonL2): 0 to 1
	gear_up = false,  -- X button, consumed after send
	gear_down = false, -- A button, consumed after send
}
local gamepad_prev = { ButtonX = false, ButtonA = false } -- edge detection
local connected_gamepad = nil

local GAMEPAD_TYPES = {
	[Enum.UserInputType.Gamepad1] = true,
	[Enum.UserInputType.Gamepad2] = true,
	[Enum.UserInputType.Gamepad3] = true,
	[Enum.UserInputType.Gamepad4] = true,
	[Enum.UserInputType.Gamepad5] = true,
	[Enum.UserInputType.Gamepad6] = true,
	[Enum.UserInputType.Gamepad7] = true,
	[Enum.UserInputType.Gamepad8] = true,
}

local function isGamepadType(userInputType)
	return GAMEPAD_TYPES[userInputType] == true
end

local function triggerValue(position)
	-- Roblox exposes standard triggers on Z. A few controller backends expose
	-- them on Y instead; use the positive component without abs(), otherwise a
	-- resting negative axis would look like a permanently pressed trigger.
	return math.clamp(math.max(position.Z, position.Y, 0), 0, 1)
end

-- Diagnostics: print once per connection so Studio Output shows whether the
-- controller is visible to Roblox and which KeyCodes/axes it exposes.
local gamepad_diag_printed = false

local function readGamepad()
	local connected = UserInputService:GetConnectedGamepads()
	if #connected == 0 then
		connected_gamepad = nil
		gamepad_state.steer = 0
		gamepad_state.throttle = 0
		gamepad_state.brake = 0
		gamepad_prev.ButtonX = false
		gamepad_prev.ButtonA = false
		gamepad_diag_printed = false
		return
	end

	-- Use the actual connected identifier instead of assuming Gamepad1. This
	-- matters when another controller was connected earlier in the session.
	connected_gamepad = connected[1]
	local ok, state = pcall(function()
		return UserInputService:GetGamepadState(connected_gamepad)
	end)
	if not ok or type(state) ~= "table" then return end

	-- Start from neutral each poll so a disconnected/omitted axis cannot leave
	-- the last throttle, brake, or steering value latched.
	gamepad_state.steer = 0
	gamepad_state.throttle = 0
	gamepad_state.brake = 0

	if not gamepad_diag_printed and #state > 0 then
		gamepad_diag_printed = true
		local parts = {}
		for _, obj in state do
			table.insert(parts, string.format("%s P=(%.2f,%.2f,%.2f)", tostring(obj.KeyCode), obj.Position.X, obj.Position.Y, obj.Position.Z))
		end
		print("[KemudiClient] " .. tostring(connected_gamepad) .. " state (" .. #state .. " objects): " .. table.concat(parts, " | "))
	end

	for _, obj in state do
		local code = obj.KeyCode
		if code == Enum.KeyCode.Thumbstick1 then
			gamepad_state.steer = math.clamp(obj.Position.X, -1, 1)
		elseif code == Enum.KeyCode.ButtonL2 then
			gamepad_state.brake = triggerValue(obj.Position)
		elseif code == Enum.KeyCode.ButtonR2 then
			gamepad_state.throttle = triggerValue(obj.Position)
		elseif code == Enum.KeyCode.ButtonX then
			local pressed = obj.Position.Z > 0.5 or obj.Position.Y > 0.5
			if pressed and not gamepad_prev.ButtonX then gamepad_state.gear_up = true end
			gamepad_prev.ButtonX = pressed
		elseif code == Enum.KeyCode.ButtonA then
			local pressed = obj.Position.Z > 0.5 or obj.Position.Y > 0.5
			if pressed and not gamepad_prev.ButtonA then gamepad_state.gear_down = true end
			gamepad_prev.ButtonA = pressed
		end
	end
end

-- Polling above is authoritative for vehicle controls. Keep this event path
-- for controllers that report button presses before their first poll.
UserInputService.InputChanged:Connect(function(gameInput)
	if not isGamepadType(gameInput.UserInputType) then return end
	local code = gameInput.KeyCode
	if code == Enum.KeyCode.Thumbstick1 then
		gamepad_state.steer = math.clamp(gameInput.Position.X, -1, 1)
	elseif code == Enum.KeyCode.ButtonL2 then
		gamepad_state.brake = triggerValue(gameInput.Position)
	elseif code == Enum.KeyCode.ButtonR2 then
		gamepad_state.throttle = triggerValue(gameInput.Position)
	end
end)

local reject_beep = Instance.new("Sound")
reject_beep.Name = "KemudiTransmissionRejectBeep"
reject_beep.SoundId = "rbxasset://sounds/electronicpingshort.wav"
reject_beep.Volume = 0.45
reject_beep.Parent = SoundService

local function playTransmissionRejectAlert()
	SoundService:PlayLocalSound(reject_beep)
	task.delay(0.12, function()
		if reject_beep.Parent then SoundService:PlayLocalSound(reject_beep) end
	end)
end

local function isDriving()
	local character = Players.LocalPlayer.Character
	local humanoid = if character then character:FindFirstChildOfClass("Humanoid") else nil
	local seat = if humanoid then humanoid.SeatPart else nil
	return seat ~= nil and seat:GetAttribute("KemudiDriverSeat") == true
end

UserInputService.InputBegan:Connect(function(gameInput, processed)
	if gameInput.UserInputType == Enum.UserInputType.Keyboard then
		local selectorKey = gameInput.KeyCode == Enum.KeyCode.LeftShift or gameInput.KeyCode == Enum.KeyCode.LeftControl
		if processed and not selectorKey then return end
		if gameInput.KeyCode == Enum.KeyCode.F and isDriving() then
			EngineToggleEvent:FireServer()
			return
		end
		keys_held[gameInput.KeyCode] = true
	elseif isGamepadType(gameInput.UserInputType) then
		-- Button events are an edge-trigger fallback; polling in readGamepad()
		-- remains authoritative and catches controllers with incomplete events.
		if gameInput.KeyCode == Enum.KeyCode.ButtonX then
			gamepad_state.gear_up = true
		elseif gameInput.KeyCode == Enum.KeyCode.ButtonA then
			gamepad_state.gear_down = true
		end
	end
end)

UserInputService.InputEnded:Connect(function(gameInput)
	if gameInput.UserInputType == Enum.UserInputType.Keyboard then
		keys_held[gameInput.KeyCode] = false
	end
end)
local function processInput(dt)
	readGamepad()
	-- Throttle/brake: gamepad analog preferred, keyboard digital fallback
	local accel = if gamepad_state.throttle > 0.05 then gamepad_state.throttle else (keys_held[Enum.KeyCode.W] and 1 or 0)
	local brake = if gamepad_state.brake > 0.05 then gamepad_state.brake else (keys_held[Enum.KeyCode.S] and 1 or 0)

	-- Steering: gamepad analog preferred, keyboard digital fallback
	local steer_target = 0
	if math.abs(gamepad_state.steer) > 0.08 then
		steer_target = math.clamp(gamepad_state.steer, -1, 1)
	else
		if keys_held[Enum.KeyCode.A] then steer_target = -1 end
		if keys_held[Enum.KeyCode.D] then steer_target = 1 end
	end

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
	local driving = isDriving()
	-- Edge-triggered commands are latched until the next 20 Hz network send.
	-- Clearing them here would lose most key presses between Heartbeats.
	if driving and keys_held[Enum.KeyCode.E] then
		input.gear_up = true
		input.manual_shift_up = true
		keys_held[Enum.KeyCode.E] = false -- consume
	end
	if driving and keys_held[Enum.KeyCode.Q] then
		input.gear_down = true
		input.manual_shift_down = true
		keys_held[Enum.KeyCode.Q] = false
	end
	if driving and gamepad_state.gear_up then
		input.gear_up = true
		input.manual_shift_up = true
		gamepad_state.gear_up = false
	end
	if driving and gamepad_state.gear_down then
		input.gear_down = true
		input.manual_shift_down = true
		gamepad_state.gear_down = false
	end
	if driving and keys_held[Enum.KeyCode.LeftShift] then
		input.selector_up = true
		keys_held[Enum.KeyCode.LeftShift] = false
	end
	if driving and keys_held[Enum.KeyCode.LeftControl] then
		input.selector_down = true
		keys_held[Enum.KeyCode.LeftControl] = false
	end
	if not driving then
		input.gear_up = false
		input.gear_down = false
		input.manual_shift_up = false
		input.manual_shift_down = false
		input.selector_up = false
		input.selector_down = false
		gamepad_state.gear_up = false
		gamepad_state.gear_down = false
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
-- Keep the complete HUD hidden until the local player occupies the authored
-- driver seat. The state transition below is the single visibility boundary.
hud_gui.Enabled = false

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
	UDim2.fromOffset(300, 190),
	UDim2.new(1, -22, 1, -22),
	Vector2.new(1, 1)
)
makeLabel(dashboard, "Title", UDim2.fromOffset(12, 8), UDim2.fromOffset(270, 20), "KEMUDI // DRIVER HUD", 15)
local speed_label = makeLabel(dashboard, "Speed", UDim2.fromOffset(12, 33), UDim2.fromOffset(170, 42), "000 km/h", 30)
local gear_label = makeLabel(dashboard, "Gear", UDim2.fromOffset(190, 36), UDim2.fromOffset(95, 30), "GEAR N", 18)
local status_label = makeLabel(dashboard, "Status", UDim2.fromOffset(12, 78), UDim2.fromOffset(270, 22), "ENGINE OFF // PRESS F", 14)
local input_label = makeLabel(dashboard, "Inputs", UDim2.fromOffset(12, 105), UDim2.fromOffset(270, 42), "THR 0%  BRK 0%  STR 0%", 13)
local transmission_warning_label = makeLabel(dashboard, "TransmissionWarning", UDim2.fromOffset(12, 145), UDim2.fromOffset(275, 34), "", 12)
transmission_warning_label.TextColor3 = Color3.fromRGB(255, 190, 80)
transmission_warning_label.TextWrapped = true
transmission_warning_label.Visible = false

local transmission_warning_token = 0
local transmission_warning_messages = {
	projected_overrev = "UNSAFE DOWNSHIFT // OVER-REV PROTECTION",
	vehicle_moving = "SHIFT REJECTED // VEHICLE MOVING",
	brake_interlock = "SHIFT REJECTED // HOLD BRAKE",
	shift_in_progress = "SHIFT REJECTED // SHIFT IN PROGRESS",
	transmission_failed = "TRANSMISSION FAILED // NEUTRAL",
	gear_limit = "SHIFT REJECTED // GEAR LIMIT",
	neutral_engagement_damage = "NEUTRAL ENGAGEMENT // DAMAGE RISK",
}

local function showTransmissionRejectWarning(reason, code, kind)
	if kind ~= "warning" then playTransmissionRejectAlert() end
	transmission_warning_token += 1
	local token = transmission_warning_token
	local message = transmission_warning_messages[reason] or string.format("SHIFT REJECTED // CODE %s", tostring(code or 0))
	transmission_warning_label.Text = "TRANS WARNING // " .. message
	transmission_warning_label.Visible = true
	task.delay(2.5, function()
		if token == transmission_warning_token and transmission_warning_label.Parent then
			transmission_warning_label.Visible = false
		end
	end)
end

TransmissionAlertEvent.OnClientEvent:Connect(showTransmissionRejectWarning)

local obd_panel = makePanel(
	hud_gui,
	"OBD",
	UDim2.fromOffset(430, 430),
	UDim2.fromOffset(22, 22)
)
makeLabel(obd_panel, "Title", UDim2.fromOffset(12, 8), UDim2.fromOffset(400, 20), "OBD-II // FULL TELEMETRY", 15)
local obd_status_label = makeLabel(obd_panel, "LinkStatus", UDim2.fromOffset(290, 8), UDim2.fromOffset(125, 20), "LINK CONNECTED", 11)
obd_status_label.TextXAlignment = Enum.TextXAlignment.Right

local tab_bar = Instance.new("Frame")
tab_bar.Name = "Tabs"
tab_bar.Position = UDim2.fromOffset(10, 34)
tab_bar.Size = UDim2.new(1, -20, 0, 28)
tab_bar.BackgroundTransparency = 1
tab_bar.Parent = obd_panel

local content = Instance.new("Frame")
content.Name = "Content"
content.Position = UDim2.fromOffset(10, 68)
content.Size = UDim2.new(1, -20, 1, -78)
content.BackgroundTransparency = 1
content.Parent = obd_panel

local function telemetryValue(data, index)
	local value = data[index + 1]
	return if type(value) == "number" and value == value then value else 0
end

local function gearText(value)
	local gear = math.floor(value + 0.5)
	if gear == -2 then return "P" end
	if gear < 0 then return "R" end
	if gear == 0 then return "N" end
	return tostring(gear)
end

local function percentText(value)
	return string.format("%3d%%", math.clamp(math.floor(value * 100 + 0.5), 0, 100))
end

local function signedPercentText(value)
	return string.format("%+4d%%", math.floor(value * 100 + (if value >= 0 then 0.5 else -0.5)))
end

local function booleanText(value)
	return if value > 0.5 then "ON" else "OFF"
end

local function stageText(value)
	local stage = math.floor(value + 0.5)
	return if stage == 1 then "ACCESSORY" elseif stage == 2 then "RUNNING" else "OFF"
end

local function shiftPhaseText(value)
	local phase = math.floor(value + 0.5)
	return ({ "IDLE", "DISENGAGING", "NEUTRAL", "ENGAGING" })[phase + 1] or tostring(phase)
end

local function tcmStateText(value)
	return ({ "NORMAL", "SPORT", "WINTER", "MANUAL", "LIMP", "FAULT", "OFF" })[math.floor(value + 0.5) + 1] or "UNKNOWN"
end

local function selectorText(data)
	local gear = math.floor(telemetryValue(data, Contract.T.GEAR) + 0.5)
	if gear == -2 then return "P" end
	if gear == -1 then return "R" end
	if gear == 0 then return "N" end
	local tcmState = telemetryValue(data, Contract.T.TCM_STATE)
	return if math.floor(tcmState + 0.5) == 3 then "M/S" else "D"
end

local function formatMetric(metric, value)
	local formatted = if metric.format == "percent" then percentText(value)
		elseif metric.format == "signed_percent" then signedPercentText(value)
		elseif metric.format == "bool" then booleanText(value)
		elseif metric.format == "gear" then gearText(value)
		elseif metric.format == "selector" then selectorText(latest_telemetry or {})
		elseif metric.format == "stage" then stageText(value)
		elseif metric.format == "phase" then shiftPhaseText(value)
		elseif metric.format == "tcm_state" then tcmStateText(value)
		elseif metric.format == "integer" then tostring(math.floor(value + 0.5))
		elseif metric.format == "rpm" then string.format("%d rpm", math.max(0, math.floor(value + 0.5)))
		elseif metric.format == "celsius" then string.format("%.1f C", value)
		elseif metric.format == "kmh" then string.format("%.1f km/h", value)
		elseif metric.format == "mps" then string.format("%.2f m/s", value)
		elseif metric.format == "radians" then string.format("%.2f rad/s", value)
		elseif metric.format == "seconds" then string.format("%.2f s", value)
		else string.format("%.3f", value)
	return string.format("%-24s %s", metric.label, formatted)
end

local tabs = {
	{
		name = "DRIVE",
		metrics = {
			{ label = "Speed", index = Contract.T.SPEED_KMH, format = "kmh" },
			{ label = "Speed SI", index = Contract.T.SPEED_MPS, format = "mps" },
			{ label = "Engine RPM", index = Contract.T.RPM, format = "rpm" },
			{ label = "Current gear", index = Contract.T.GEAR, format = "gear" },
			{ label = "Selector", index = Contract.T.GEAR, format = "selector" },
			{ label = "Throttle", index = Contract.T.THROTTLE, format = "percent" },
			{ label = "Brake", index = Contract.T.BRAKE, format = "percent" },
			{ label = "Steering", index = Contract.T.STEERING, format = "signed_percent" },
			{ label = "Clutch", index = Contract.T.CLUTCH, format = "percent" },
			{ label = "Wheel speed", index = Contract.T.WHEEL_SPEED, format = "radians" },
			{ label = "Drive torque", index = Contract.T.DRIVE_TORQUE },
			{ label = "Engine running", index = Contract.T.ENGINE_RUNNING, format = "bool" },
			{ label = "Engine stage", index = Contract.T.ENGINE_STAGE, format = "stage" },
			{ label = "Shift phase", index = Contract.T.SHIFT_PHASE, format = "phase" },
			{ label = "Simulation time", index = Contract.T.SIM_TIME, format = "seconds" },
			{ label = "Terrain height", index = Contract.T.TERRAIN_HEIGHT },
			{ label = "Wheel contacts", index = Contract.T.CONTACTS, format = "integer" },
		},
	},
	{
		name = "ENGINE",
		metrics = {
			{ label = "Coolant", index = Contract.T.COOLANT, format = "celsius" },
			{ label = "Oil temperature", index = Contract.T.OIL_TEMP, format = "celsius" },
			{ label = "Oil pressure", index = Contract.T.OIL_PRESSURE },
			{ label = "Fuel level", index = Contract.T.FUEL },
			{ label = "Fuel remaining", index = Contract.T.FUEL_PERCENT, format = "percent" },
			{ label = "Power derate", index = Contract.T.DERATE, format = "percent" },
			{ label = "Engine damage", index = Contract.T.DAMAGE, format = "percent" },
			{ label = "Engine warning", index = Contract.T.ENGINE_WARNING, format = "bool" },
			{ label = "Lugging stress", index = Contract.T.LUGGING_STRESS, format = "percent" },
			{ label = "Overrev stress", index = Contract.T.OVERREV_STRESS, format = "percent" },
			{ label = "Impact severity", index = Contract.T.IMPACT_SEVERITY, format = "percent" },
		},
	},
	{
		name = "TRANS",
		metrics = {
			{ label = "TCM state", index = Contract.T.TCM_STATE, format = "tcm_state" },
			{ label = "Line pressure", index = Contract.T.TCM_LINE_PRESSURE },
			{ label = "Lockup", index = Contract.T.TCM_LOCKUP, format = "bool" },
			{ label = "Requested gear", index = Contract.T.REQUESTED_GEAR, format = "gear" },
			{ label = "Input shaft", index = Contract.T.INPUT_SHAFT_RPM, format = "rpm" },
			{ label = "Output shaft", index = Contract.T.OUTPUT_SHAFT_RPM, format = "rpm" },
			{ label = "Converter coupling", index = Contract.T.CONVERTER_COUPLING, format = "percent" },
			{ label = "Transmission temp", index = Contract.T.TRANS_TEMP, format = "celsius" },
			{ label = "Transmission damage", index = Contract.T.DAMAGE, format = "percent" },
			{ label = "Sensor input RPM", index = Contract.T.TCM_SENSOR_INPUT_RPM, format = "rpm" },
			{ label = "Sensor speed", index = Contract.T.TCM_SENSOR_SPEED_KMH, format = "kmh" },
			{ label = "Sensor age", index = Contract.T.TCM_SENSOR_AGE, format = "seconds" },
			{ label = "Shift latency", index = Contract.T.TCM_SHIFT_LATENCY, format = "seconds" },
			{ label = "Torque reduction", index = Contract.T.TCM_TORQUE_REDUCTION, format = "percent" },
			{ label = "Fail-safe gear", index = Contract.T.TCM_FAIL_SAFE_GEAR, format = "gear" },
			{ label = "Fault mask", index = Contract.T.TCM_FAULT_MASK, format = "integer" },
			{ label = "Diagnostic code", index = Contract.T.TCM_DIAGNOSTIC_CODE, format = "integer" },
		},
	},
	{
		name = "CHASSIS",
		metrics = {
			{ label = "Suspension load", index = Contract.T.SUSPENSION_LOAD },
			{ label = "Average grip", index = Contract.T.GRIP, format = "percent" },
			{ label = "Slip ratio", index = Contract.T.SLIP_RATIO, format = "percent" },
			{ label = "Yaw error", index = Contract.T.YAW_ERROR },
			{ label = "Rut depth", index = Contract.T.RUT_DEPTH },
			{ label = "Broken beams", index = Contract.T.BROKEN_BEAMS, format = "integer" },
			{ label = "Deformation depth", index = Contract.T.DEFORMATION_DEPTH },
			{ label = "Broken parts", index = Contract.T.BROKEN_PARTS, format = "integer" },
			{ label = "Damage zone", index = Contract.T.DAMAGE_ZONE, format = "integer" },
			{ label = "Wheel speed FL", index = Contract.T.WHEEL_SPEED_FL, format = "radians" },
			{ label = "Wheel speed FR", index = Contract.T.WHEEL_SPEED_FR, format = "radians" },
			{ label = "Wheel speed RL", index = Contract.T.WHEEL_SPEED_RL, format = "radians" },
			{ label = "Wheel speed RR", index = Contract.T.WHEEL_SPEED_RR, format = "radians" },
			{ label = "Suspension FL", index = Contract.T.SUSP_COMPRESSION_FL },
			{ label = "Suspension FR", index = Contract.T.SUSP_COMPRESSION_FR },
			{ label = "Suspension RL", index = Contract.T.SUSP_COMPRESSION_RL },
			{ label = "Suspension RR", index = Contract.T.SUSP_COMPRESSION_RR },
			{ label = "Load FL", index = Contract.T.SUSP_LOAD_FL },
			{ label = "Load FR", index = Contract.T.SUSP_LOAD_FR },
			{ label = "Load RL", index = Contract.T.SUSP_LOAD_RL },
			{ label = "Load RR", index = Contract.T.SUSP_LOAD_RR },
		},
	},
	{
		name = "TIRES",
		metrics = {
			{ label = "Tire temperature", index = Contract.T.TIRE_TEMP, format = "celsius" },
			{ label = "Tire wear", index = Contract.T.TIRE_WEAR, format = "percent" },
			{ label = "Temp FL", index = Contract.T.TIRE_TEMP_FL, format = "celsius" },
			{ label = "Temp FR", index = Contract.T.TIRE_TEMP_FR, format = "celsius" },
			{ label = "Temp RL", index = Contract.T.TIRE_TEMP_RL, format = "celsius" },
			{ label = "Temp RR", index = Contract.T.TIRE_TEMP_RR, format = "celsius" },
			{ label = "Wear FL", index = Contract.T.TIRE_WEAR_FL, format = "percent" },
			{ label = "Wear FR", index = Contract.T.TIRE_WEAR_FR, format = "percent" },
			{ label = "Wear RL", index = Contract.T.TIRE_WEAR_RL, format = "percent" },
			{ label = "Wear RR", index = Contract.T.TIRE_WEAR_RR, format = "percent" },
		},
	},
	{
		name = "SAFETY",
		metrics = {
			{ label = "ABS", index = Contract.T.ABS_ACTIVE, format = "bool" },
			{ label = "Traction control", index = Contract.T.TC_ACTIVE, format = "bool" },
			{ label = "Stability control", index = Contract.T.VSC_ACTIVE, format = "bool" },
			{ label = "FCW", index = Contract.T.ADAS_FCW, format = "bool" },
			{ label = "AEB", index = Contract.T.ADAS_AEB, format = "bool" },
			{ label = "ADAS confidence", index = Contract.T.ADAS_CONFIDENCE, format = "percent" },
			{ label = "Target detected", index = Contract.T.ADAS_TARGET_DETECTED, format = "bool" },
			{ label = "Target distance", index = Contract.T.ADAS_TARGET_DISTANCE },
			{ label = "Time to collision", index = Contract.T.ADAS_TIME_TO_COLLISION },
			{ label = "Relative speed", index = Contract.T.ADAS_RELATIVE_SPEED },
			{ label = "AEB brake command", index = Contract.T.ADAS_BRAKE_COMMAND, format = "percent" },
			{ label = "ADAS sensor faults", index = Contract.T.ADAS_SENSOR_FAULTS, format = "integer" },
		},
	},
}

local tab_buttons = {}
local tab_pages = {}
local page_rows = {}
local active_tab = tabs[1].name

local function setActiveTab(name)
	active_tab = name
	for tabName, page in tab_pages do
		page.Visible = tabName == active_tab
	end
	for tabName, button in tab_buttons do
		local active = tabName == active_tab
		button.BackgroundColor3 = if active then Color3.fromRGB(35, 115, 155) else Color3.fromRGB(18, 30, 42)
		button.TextColor3 = if active then Color3.fromRGB(255, 255, 255) else Color3.fromRGB(175, 210, 225)
	end
end

for tabIndex, tab in tabs do
	local button = Instance.new("TextButton")
	button.Name = tab.name
	button.Size = UDim2.new(1 / #tabs, -3, 1, 0)
	button.Position = UDim2.new((tabIndex - 1) / #tabs, 0, 0, 0)
	button.BackgroundColor3 = Color3.fromRGB(18, 30, 42)
	button.BorderSizePixel = 0
	button.AutoButtonColor = false
	button.Font = Enum.Font.Code
	button.TextSize = 11
	button.Text = tab.name
	button.Parent = tab_bar
	button.Activated:Connect(function() setActiveTab(tab.name) end)
	tab_buttons[tab.name] = button

	local page = Instance.new("ScrollingFrame")
	page.Name = tab.name .. "Page"
	page.Size = UDim2.fromScale(1, 1)
	page.BackgroundTransparency = 1
	page.BorderSizePixel = 0
	page.ScrollBarThickness = 5
	page.ScrollBarImageColor3 = Color3.fromRGB(80, 170, 220)
	page.CanvasSize = UDim2.fromOffset(0, #tab.metrics * 22 + 8)
	page.ScrollingDirection = Enum.ScrollingDirection.Y
	page.Parent = content
	local layout = Instance.new("UIListLayout")
	layout.Padding = UDim.new(0, 2)
	layout.SortOrder = Enum.SortOrder.LayoutOrder
	layout.Parent = page

	tab_pages[tab.name] = page
	page_rows[tab.name] = {}
	for metricIndex, metric in tab.metrics do
		local row = makeLabel(page, metric.label, UDim2.fromOffset(8, 0), UDim2.new(1, -16, 0, 20), metric.label .. "  --", 13)
		row.LayoutOrder = metricIndex
		row.TextYAlignment = Enum.TextYAlignment.Center
		table.insert(page_rows[tab.name], { label = row, metric = metric })
	end
end
setActiveTab(active_tab)

local function updateHud(data)
	local speed = telemetryValue(data, Contract.T.SPEED_KMH)
	local gear = telemetryValue(data, Contract.T.GEAR)
	local throttle = telemetryValue(data, Contract.T.THROTTLE)
	local brake = telemetryValue(data, Contract.T.BRAKE)
	local steering = telemetryValue(data, Contract.T.STEERING)
	local engineStage = math.floor(telemetryValue(data, Contract.T.ENGINE_STAGE) + 0.5)

	speed_label.Text = string.format("%03d km/h", math.max(0, math.floor(speed + 0.5)))
	gear_label.Text = "GEAR " .. gearText(gear)
	local engineText = if engineStage == 1 then "ACCESSORY" elseif engineStage == 2 then "ENGINE ON" else "ENGINE OFF"
	status_label.Text = if isDriving() then engineText .. (if engineStage == 2 then " // DRIVE READY" else " // PRESS F") else "EXITED // PRESS ENTER"
	input_label.Text = string.format("THR %02d%%  BRK %02d%%  STR %+03d%%", math.floor(throttle * 100 + 0.5), math.floor(brake * 100 + 0.5), math.floor(steering * 100 + (if steering >= 0 then 0.5 else -0.5)))
	obd_status_label.Text = if telemetryValue(data, Contract.T.ENGINE_WARNING) > 0 or telemetryValue(data, Contract.T.TCM_FAULT_MASK) > 0
		then "LINK // WARNING"
		else "LINK CONNECTED"

	for tabName, rows in page_rows do
		for _, row in rows do
			row.label.Text = formatMetric(row.metric, telemetryValue(data, row.metric.index))
		end
	end
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
		hud_gui.Enabled = driving
		if not driving then
			transmission_warning_token += 1
			transmission_warning_label.Visible = false
		end
		last_driving_state = driving
	end

	send_timer += dt
	if send_timer >= send_interval then
		send_timer = 0
		if isDriving() then
			ControlsEvent:FireServer(input)
			-- These are commands, not held analog values. Clear only after the
			-- command has crossed the network boundary.
			input.gear_up = false
			input.gear_down = false
			input.manual_shift_up = false
			input.manual_shift_down = false
			input.selector_up = false
			input.selector_down = false
		end
	end
end)

-- ============================================================
-- Configurator UI (real-time vehicle tuning)
-- ============================================================

local ConfiguratorUI = require(ReplicatedStorage:WaitForChild("KemudiEngine"):WaitForChild("configurator"):WaitForChild("ui"))
ConfiguratorUI.init()

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
