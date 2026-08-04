--[[
	Vehicle Asset Loader — loads vehicle specifications from Lua files in assets/vehicles/
	Used as an alternative to the Roblox Model-based loader for programmatic vehicle configs.
]]

local Loader = {}

-- Resolve assets/vehicles/ path - works in Roblox (ModuleScript) and Lune (file system)
local function getAssetsVehiclesPath()
	-- In Roblox via Rojo: the .luau files are synced as ModuleScripts under ReplicatedStorage.KemudiEngine.assets.vehicles
	local ReplicatedStorage = game:GetService("ReplicatedStorage")
	local kemudiEngine = ReplicatedStorage:FindFirstChild("KemudiEngine")
	if kemudiEngine then
		local assetsFolder = kemudiEngine:FindFirstChild("assets")
		if assetsFolder then
			local vehiclesFolder = assetsFolder:FindFirstChild("vehicles")
			if vehiclesFolder then
				return vehiclesFolder
			end
		end
	end
	
	-- Fallback for Lune/testing: try to find via file system
	-- This won't work in Roblox but helps with testing
	return nil
end

-- Load a vehicle config by ID from assets/vehicles/*.luau ModuleScripts
function Loader.loadFromLua(vehicleId)
	-- Try Roblox ModuleScript first (Rojo workflow: ReplicatedStorage.KemudiEngine.assets.vehicles)
	local ReplicatedStorage = game:GetService("ReplicatedStorage")
	local kemudiEngine = ReplicatedStorage:FindFirstChild("KemudiEngine")
	if kemudiEngine then
		local assetsFolder = kemudiEngine:FindFirstChild("assets")
		if assetsFolder then
			local vehiclesFolder = assetsFolder:FindFirstChild("vehicles")
			if vehiclesFolder then
				local moduleScript = vehiclesFolder:FindFirstChild(vehicleId)
				if moduleScript and moduleScript:IsA("ModuleScript") then
					local ok, config = pcall(require, moduleScript)
					if ok then
						return config
					else
						return nil, "Failed to load vehicle config: " .. tostring(config)
					end
				end
			end
		end
	end
	
	-- Fallback: try direct path under KemudiEngine.runtime.vehicle_configs (when assets folder not set up)
	if kemudiEngine then
		local runtimeFolder = kemudiEngine:FindFirstChild("runtime")
		if runtimeFolder then
			local configFolder = runtimeFolder:FindFirstChild("vehicle_configs")
			if configFolder then
				local moduleScript = configFolder:FindFirstChild(vehicleId)
				if moduleScript and moduleScript:IsA("ModuleScript") then
					local ok, config = pcall(require, moduleScript)
					if ok then
						return config
					else
						return nil, "Failed to load vehicle config: " .. tostring(config)
					end
				end
			end
		end
	end
	
	-- Fallback for Lune/testing: try to load from file system
	local success, config = pcall(function()
		local path = "assets/vehicles/" .. vehicleId .. ".luau"
		return dofile(path)
	end)
	if success and config then
		return config
	end
	
	return nil, "Vehicle config not found: " .. vehicleId .. " (checked Roblox ModuleScripts and file system)"
end

-- Convert Lua vehicle config to the definition format expected by Runtime.new
function Loader.configToDefinition(config, vehicleId)
	-- Derive mass from vehicle config
	local vehicleMass = config.test_mass or config.mass or 1500
	
	-- Build minimal definition with defaults for fields normally loaded from Model
	return {
		model = nil, -- Not a Roblox Model
		nodeDefs = {}, -- Will be populated by Runtime from suspension config
		beamDefs = {},
		renderParts = {},
		wheels = {},
		driverSeat = nil,
		rootPart = nil,
		vehicleMass = vehicleMass,
		studsPerMeter = 1 / 0.28,
		driveLayout = "AWD",
		simulation_config = {
			aero = config.aero or {},
			brakes = config.brakes or {},
			wheel_dynamics = config.tire or {},
			suspension_kinematics = config.suspension or {},
			engine_breathing = config.engine_breathing or {},
			angular = {
				inertia = config.inertia or { Ixx = 1000, Iyy = 4000, Izz = 4500 }
			},
			solver = {},
			environment = {},
			contact = {},
			terrain = {},
			collision = {},
			bushings = {}
		},
		drivetrain = {
			engine = config.engine,
			transmission = config.transmission,
			tcm = config.tcm,
			transfer_case = config.transfer_case,
			front_differential = config.front_differential,
			rear_differential = config.rear_differential,
			turbo = config.turbo,
			engine_thermal = config.engine_thermal,
		}
	}
end

return Loader