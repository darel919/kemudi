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