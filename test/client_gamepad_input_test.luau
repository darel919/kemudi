-- Structural contract for the Roblox gamepad input boundary.
-- Run: lune run test/client_gamepad_input_test.luau
-- Physical controller behavior still requires a Roblox Studio playtest.

local fs = require("@lune/fs")
local source = fs.readFile("src/runtime/client.luau")

local function mustContain(text, message)
	assert(source:find(text, 1, true), message .. "\nmissing: " .. text)
end

-- Roblox input is polled from the connected device, not only from events.
mustContain("UserInputService:GetConnectedGamepads()", "gamepad connection polling is required")
mustContain("UserInputService:GetGamepadState(connected_gamepad)", "the connected gamepad state must be polled")
mustContain("local connected_gamepad = nil", "the active gamepad identifier must be tracked")

-- Requested bindings.
mustContain("Enum.KeyCode.ButtonL2", "LT brake binding is missing")
mustContain("Enum.KeyCode.ButtonR2", "RT throttle binding is missing")
mustContain("Enum.KeyCode.Thumbstick1", "left-stick steering binding is missing")
mustContain("Enum.KeyCode.ButtonX", "X upshift binding is missing")
mustContain("Enum.KeyCode.ButtonA", "A downshift binding is missing")

-- Analog values must reach the normalized control table sent to the server.
mustContain("input.steering = steer_target", "steering is not forwarded")
mustContain("input.throttle = accel", "throttle is not forwarded")
mustContain("input.brake = brake", "brake is not forwarded")
mustContain("input.manual_shift_up = true", "upshift is not forwarded")
mustContain("input.manual_shift_down = true", "downshift is not forwarded")

-- Do not accidentally treat a resting negative trigger axis as full input.
mustContain("math.max(position.Z, position.Y, 0)", "trigger normalization must ignore negative rest values")

-- Right-stick camera input is intentionally not consumed by the vehicle handler;
-- Roblox's default camera controller owns Thumbstick2.
assert(not source:find("input%.camera", 1), "vehicle input must not consume the right-stick camera")

print("client gamepad input contract passed")
