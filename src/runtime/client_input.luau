--[[

	Client input — keyboard, gamepad polling, processInput, isDriving.
	Extracted from client.luau for maintainability.
	ModuleScript required by the client LocalScript.

]]

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

local function readGamepad(UserInputService)
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

local local_player = nil
local EngineToggleEvent = nil

local function isDriving()
	if not local_player then return false end
	local character = local_player.Character
	local humanoid = if character then character:FindFirstChildOfClass("Humanoid") else nil
	local seat = if humanoid then humanoid.SeatPart else nil
	return seat ~= nil and seat:GetAttribute("KemudiDriverSeat") == true
end

local function processInput(dt, UserInputService)
	readGamepad(UserInputService)
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

local function init(params)
	local_player = params.localPlayer
	EngineToggleEvent = params.EngineToggleEvent
	local UserInputService = params.UserInputService

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
end

return {
	input = input,
	processInput = processInput,
	isDriving = isDriving,
	init = init,
}
