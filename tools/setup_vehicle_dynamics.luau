--!strict
-- Kemudi Studio dynamics authoring utility.
--
-- Run once from the Roblox Studio Command Bar with one prepared vehicle Model
-- selected under ServerStorage.KemudiVehicles. This writes vehicle-owned
-- Transmission, TCM, and Suspension content. Runtime code does not use these
-- values as defaults and will reject assets that omit them.
--
-- These values are an initial calibration for the selected pickup only. Review
-- them per vehicle in Studio before Play mode; do not copy them into runtime.

local Selection = game:GetService("Selection")
local ServerStorage = game:GetService("ServerStorage")

local selected = Selection:Get()
assert(#selected == 1, "Select exactly one prepared vehicle Model before running setup")
local vehicle = selected[1]
assert(vehicle:IsA("Model"), "The selected Instance must be a Model")
local vehicles = ServerStorage:FindFirstChild("KemudiVehicles")
assert(vehicles and vehicles:IsA("Folder"), "Expected ServerStorage.KemudiVehicles")
assert(vehicle:IsDescendantOf(vehicles), "The vehicle must be under ServerStorage.KemudiVehicles")

local spec = vehicle:FindFirstChild("VehicleSpec")
local wheels = vehicle:FindFirstChild("Wheels")
assert(spec and spec:IsA("Folder"), "VehicleSpec folder is required")
assert(wheels and wheels:IsA("Folder"), "Wheels folder is required")
assert(not spec:FindFirstChild("Transmission"), "Transmission already exists; edit authored values instead of rerunning")
assert(not spec:FindFirstChild("Suspension"), "Suspension already exists; edit authored values instead of rerunning")

local TRANSMISSION = {
	Mode = "Automatic",
	FinalDrive = 3.15,
	ReverseRatio = -4.87,
	ClutchEngagement = 1.0,
	ShiftDelay = 0.25,
	GearRatios = { 4.70, 2.99, 2.15, 1.77, 1.52, 1.28, 1.00, 0.85, 0.69, 0.64 },
}

local ENGINE = {
	IdleRPM = 700,
	RedlineRPM = 6500,
	RevLimiterRPM = 6800,
	ThrottleResponse = 3.2,
	EngineBraking = 42,
	TorqueCurve = {
		{ RPM = 700, Torque = 300 },
		{ RPM = 1200, Torque = 360 },
		{ RPM = 2000, Torque = 430 },
		{ RPM = 3000, Torque = 475 },
		{ RPM = 4000, Torque = 500 },
		{ RPM = 5000, Torque = 470 },
		{ RPM = 6000, Torque = 410 },
		{ RPM = 6500, Torque = 340 },
	},
}

local TCM = {
	Enabled = true,
	UpshiftRPM = { 5600, 5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000, 0 },
	DownshiftRPM = { 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 0 },
	ThrottleShiftFactor = 0.20,
	KickdownThreshold = 0.78,
	LockupSpeedThreshold = 45,
	LockupThrottleThreshold = 0.30,
	ThermalLimit = 140,
	LimpGear = 2,
	MinShiftInterval = 0.65,
}

local SUSPENSION = {
	AntiRollBarStiffness = 12000,
	BumpStopRate = 500000,
	Wheels = {
		FL = { RestLength = 0.34, Travel = 0.22, SpringRate = 30000, Damping = 4200, ReboundDamping = 3200, UnsprungMass = 18 },
		FR = { RestLength = 0.34, Travel = 0.22, SpringRate = 30000, Damping = 4200, ReboundDamping = 3200, UnsprungMass = 18 },
		RL = { RestLength = 0.36, Travel = 0.24, SpringRate = 34000, Damping = 4600, ReboundDamping = 3500, UnsprungMass = 20 },
		RR = { RestLength = 0.36, Travel = 0.24, SpringRate = 34000, Damping = 4600, ReboundDamping = 3500, UnsprungMass = 20 },
	},
}

local function makeFolder(parent: Instance, name: string): Folder
	local folder = Instance.new("Folder")
	folder.Name = name
	folder.Parent = parent
	return folder
end

local function makeNumberValue(parent: Instance, name: string, value: number)
	local numberValue = Instance.new("NumberValue")
	numberValue.Name = name
	numberValue.Value = value
	numberValue.Parent = parent
end

local engine = makeFolder(spec, "Engine")
engine:SetAttribute("IdleRPM", ENGINE.IdleRPM)
engine:SetAttribute("RedlineRPM", ENGINE.RedlineRPM)
engine:SetAttribute("RevLimiterRPM", ENGINE.RevLimiterRPM)
engine:SetAttribute("ThrottleResponse", ENGINE.ThrottleResponse)
engine:SetAttribute("EngineBraking", ENGINE.EngineBraking)
local torqueCurve = makeFolder(engine, "TorqueCurve")
for index, point in ipairs(ENGINE.TorqueCurve) do
	local pointFolder = makeFolder(torqueCurve, tostring(index))
	pointFolder:SetAttribute("RPM", point.RPM)
	pointFolder:SetAttribute("Torque", point.Torque)
end

local transmission = makeFolder(spec, "Transmission")
transmission:SetAttribute("Mode", TRANSMISSION.Mode)
transmission:SetAttribute("FinalDrive", TRANSMISSION.FinalDrive)
transmission:SetAttribute("ReverseRatio", TRANSMISSION.ReverseRatio)
transmission:SetAttribute("ClutchEngagement", TRANSMISSION.ClutchEngagement)
transmission:SetAttribute("ShiftDelay", TRANSMISSION.ShiftDelay)
local gearRatios = makeFolder(transmission, "GearRatios")
for index, ratio in ipairs(TRANSMISSION.GearRatios) do
	makeNumberValue(gearRatios, tostring(index), ratio)
end

local tcm = makeFolder(transmission, "TCM")
tcm:SetAttribute("Enabled", TCM.Enabled)
tcm:SetAttribute("ThrottleShiftFactor", TCM.ThrottleShiftFactor)
tcm:SetAttribute("KickdownThreshold", TCM.KickdownThreshold)
tcm:SetAttribute("LockupSpeedThreshold", TCM.LockupSpeedThreshold)
tcm:SetAttribute("LockupThrottleThreshold", TCM.LockupThrottleThreshold)
tcm:SetAttribute("ThermalLimit", TCM.ThermalLimit)
tcm:SetAttribute("LimpGear", TCM.LimpGear)
tcm:SetAttribute("MinShiftInterval", TCM.MinShiftInterval)
local upshift = makeFolder(tcm, "UpshiftRPM")
local downshift = makeFolder(tcm, "DownshiftRPM")
for index = 1, #TRANSMISSION.GearRatios do
	makeNumberValue(upshift, tostring(index), TCM.UpshiftRPM[index] or 0)
	makeNumberValue(downshift, tostring(index), TCM.DownshiftRPM[index] or 0)
end

local suspension = makeFolder(spec, "Suspension")
suspension:SetAttribute("AntiRollBarStiffness", SUSPENSION.AntiRollBarStiffness)
suspension:SetAttribute("BumpStopRate", SUSPENSION.BumpStopRate)
for _, id in ipairs({ "FL", "FR", "RL", "RR" }) do
	local wheel = wheels:FindFirstChild(id)
	assert(wheel and wheel:IsA("Folder"), "Missing Wheels." .. id)
	local radiusStuds = wheel:GetAttribute("RadiusStuds")
	assert(type(radiusStuds) == "number" and radiusStuds > 0, "Wheels." .. id .. ".RadiusStuds must be authored first")
	local wheelSpec = SUSPENSION.Wheels[id]
	local wheelFolder = makeFolder(suspension, id)
	for name, value in pairs(wheelSpec) do
		wheelFolder:SetAttribute(name, value)
	end
	wheelFolder:SetAttribute("RadiusStuds", radiusStuds)
end

print("Kemudi dynamics authored for " .. vehicle:GetFullName())
print("Transmission: Automatic; " .. tostring(#TRANSMISSION.GearRatios) .. " forward gears; TCM enabled")
print("Suspension: four authored wheel configurations")
print("Review the Transmission, TCM, and Suspension folders per vehicle before saving.")
