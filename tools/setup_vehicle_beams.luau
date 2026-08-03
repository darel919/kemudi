--!strict
-- Kemudi Studio beam-content setup utility.
--
-- Run this once after setup_vehicle_mass_proxies.luau:
--   1. Select exactly one prepared vehicle Model in Explorer.
--   2. Confirm it is under ServerStorage.KemudiVehicles.
--   3. Paste this file's contents into the Command Bar and run it.
--
-- Beam values are authored into the selected asset as declarative content.
-- They are derived from the asset's proxy geometry and are not runtime constants.

local Selection = game:GetService("Selection")
local ServerStorage = game:GetService("ServerStorage")

local selected = Selection:Get()
assert(#selected == 1, "Select exactly one prepared vehicle Model before running beam setup")

local vehicle = selected[1]
assert(vehicle:IsA("Model"), "The selected Instance must be a Model")
local vehiclesFolder = ServerStorage:FindFirstChild("KemudiVehicles")
assert(vehiclesFolder ~= nil and vehiclesFolder:IsA("Folder"), "Expected ServerStorage.KemudiVehicles")
assert(vehicle:IsDescendantOf(vehiclesFolder), "The vehicle must be under ServerStorage.KemudiVehicles")

local physicsRig = vehicle:FindFirstChild("PhysicsRig")
assert(physicsRig ~= nil and physicsRig:IsA("Folder"), "Missing PhysicsRig")
local massProxies = physicsRig:FindFirstChild("MassProxies")
assert(massProxies ~= nil and massProxies:IsA("Folder"), "Missing PhysicsRig.MassProxies")
local spec = vehicle:FindFirstChild("VehicleSpec")
assert(spec ~= nil and spec:IsA("Folder"), "Missing VehicleSpec")
assert(spec:FindFirstChild("Beams") == nil, "Beam content already exists; refusing to overwrite it")

local totalMass = 0
for _, proxy in ipairs(massProxies:GetChildren()) do
	if proxy:IsA("BasePart") then
		local mass = proxy:GetMass()
		assert(mass == mass and mass > 0, "Every mass proxy must have positive derived mass")
		totalMass += mass
	end
end
assert(totalMass > 0, "Mass proxies must provide positive derived mass before beam setup")

local beams = Instance.new("Folder")
beams.Name = "Beams"
beams.Parent = spec

-- The first graph is a bounded four-node frame. Its stiffness scale is derived
-- from proxy geometry; individual values live in the Studio-authored asset.
local baseStiffness = math.max(totalMass, 1) * 1000
local baseDamping = math.sqrt(baseStiffness * math.max(totalMass, 1))
local baseStrength = baseStiffness * 0.2

local definitions = {
	{ name = "Beam_FL_FR", a = "Node_FL", b = "Node_FR", stiffness = baseStiffness, damping = baseDamping, strength = baseStrength },
	{ name = "Beam_FR_RR", a = "Node_FR", b = "Node_RR", stiffness = baseStiffness, damping = baseDamping, strength = baseStrength },
	{ name = "Beam_RR_RL", a = "Node_RR", b = "Node_RL", stiffness = baseStiffness, damping = baseDamping, strength = baseStrength },
	{ name = "Beam_RL_FL", a = "Node_RL", b = "Node_FL", stiffness = baseStiffness, damping = baseDamping, strength = baseStrength },
	{ name = "Beam_FL_RR", a = "Node_FL", b = "Node_RR", stiffness = baseStiffness * 0.75, damping = baseDamping, strength = baseStrength },
	{ name = "Beam_FR_RL", a = "Node_FR", b = "Node_RL", stiffness = baseStiffness * 0.75, damping = baseDamping, strength = baseStrength },
}

for _, definition in ipairs(definitions) do
	local beam = Instance.new("Folder")
	beam.Name = definition.name
	beam:SetAttribute("NodeA", definition.a)
	beam:SetAttribute("NodeB", definition.b)
	beam:SetAttribute("Stiffness", definition.stiffness)
	beam:SetAttribute("Damping", definition.damping)
	beam:SetAttribute("Strength", definition.strength)
	beam.Parent = beams
end

spec:SetAttribute("DriveLayout", "RWD")
spec:SetAttribute("BeamPolicy", "asset-authored")

print(string.format(
	"Kemudi beam content prepared: %s; %d beams derived from %.2f proxy mass units",
	beams:GetFullName(),
	#definitions,
	totalMass
))
print("Beam tuning is stored in VehicleSpec.Beams; server.luau contains no vehicle beam list.")
