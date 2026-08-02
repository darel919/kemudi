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
	-- Try .luau then .lua
	for _, ext in { ".luau", ".lua" } do
		local path = BASE .. "/" .. name .. ext
		if fs.isFile(path) then
			local ok, mod = pcall(require, path:gsub("%.luau$",""):gsub("%.lua$",""))
			if ok then return mod end
		end
	end

	-- Try init.luau inside directory
	for _, ext in { ".luau", ".lua" } do
		local path = BASE .. "/" .. name .. "/init" .. ext
		if fs.isFile(path) then
			local ok, mod = pcall(require, path:gsub("%.luau$",""):gsub("%.lua$",""))
			if ok then return mod end
		end
	end

	error("Could not load module: " .. name)
end

function Loader.fixture(name)
	local path = "/Users/darelisme/Documents/Development/kemudi.js/fixtures/engine/" .. name .. ".json"
	return serde.decode("json", fs.readFile(path))
end

return Loader
