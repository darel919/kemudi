-- Regression contract for the Roblox client HUD visibility boundary.
-- Run: lune run test/client_hud_visibility_test.luau

local fs = require("@lune/fs")

local source = fs.readFile("src/runtime/client.luau")
local contract = fs.readFile("src/contract/init.luau")

assert(source:find("hud_gui.Enabled = false", 1, true),
	"the HUD must start hidden before vehicle occupancy is established")
assert(source:find("hud_gui.Enabled = driving", 1, true),
	"the HUD must follow the local driver's occupancy state")
assert(source:find("local driving = isDriving()", 1, true),
	"HUD visibility must use the authoritative local driver-seat check")
assert(source:find("ScrollingFrame", 1, true),
	"the OBD HUD must provide scrollable diagnostic pages")
assert(source:find("TextButton", 1, true),
	"the OBD HUD must provide interactive tab controls")
assert(source:find("KemudiTransmissionAlert", 1, true),
	"the client must subscribe to authoritative transmission rejection alerts")
assert(source:find("if processed and not selectorKey then return end", 1, true),
	"LeftShift and LeftControl must not be swallowed by Roblox processed-input handling")
assert(source:find("latched until the next 20 Hz network send", 1, true),
	"one-shot selector inputs must remain latched until they are sent")
assert(source:find("command has crossed the network boundary", 1, true),
	"one-shot selector inputs must clear only after FireServer")
assert(source:find("UNSAFE DOWNSHIFT // OVER-REV PROTECTION", 1, true),
	"unsafe high-speed downshifts must have an explicit visible warning message")
assert(source:find("NEUTRAL ENGAGEMENT // DAMAGE RISK", 1, true),
	"moving neutral engagements must have an explicit damage-risk message")
assert(source:find("TransmissionWarning", 1, true),
	"the HUD must provide a dedicated transmission warning label")
assert(source:find("transmission_warning_label.Visible = true", 1, true),
	"transmission rejection alerts must make the warning label visible")

for _, tabName in { "DRIVE", "ENGINE", "TRANS", "CHASSIS", "TIRES", "SAFETY" } do
	assert(source:find('name = "' .. tabName .. '"', 1, true),
		"missing OBD HUD tab: " .. tabName)
end

local telemetryBlock = contract:match("Contract%.T = %{(.-)\n%}")
assert(telemetryBlock, "telemetry contract table must be discoverable")
for telemetryName in telemetryBlock:gmatch("([A-Z][A-Z0-9_]*)%s*=") do
	assert(source:find("Contract.T." .. telemetryName, 1, true),
		"telemetry signal is not represented in the HUD: " .. telemetryName)
end

print("client HUD visibility contract passed")