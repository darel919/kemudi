--[[
	Configurator module — aggregates all configurator sub-modules.
	Require this as `require(ReplicatedStorage.KemudiEngine.configurator)`.
--]]

local Manifest = require(script:WaitForChild("manifest"))
local Configurator = require(script:WaitForChild("vehicle_configurator"))
local Presets = require(script:WaitForChild("param_presets"))
local ServerSync = require(script:WaitForChild("server_sync"))
local UI = require(script:WaitForChild("ui"))

return {
	Manifest = Manifest,
	Configurator = Configurator,
	Presets = Presets,
	ServerSync = ServerSync,
	UI = UI,
}
