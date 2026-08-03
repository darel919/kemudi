--[[
	Module loader for Lune tests.
	Loads Luau modules via relative paths, bypassing Roblox-style requires.
--]]
local fs = require("@lune/fs")
local serde = require("@lune/serde")
local process = require("@lune/process")

local Loader = {}

local BASE = "../src"

function Loader.load(name)
	local ok, mod = pcall(require, BASE .. "/" .. name)
	if ok then return mod end
	local initOk, initMod = pcall(require, BASE .. "/" .. name .. "/init")
	if initOk then return initMod end

	error("Could not load module: " .. name)
end

function Loader.fixture(name)
	local path = "fixtures/engine/" .. name .. ".json"
	return serde.decode("json", fs.readFile(path))
end

return Loader
