--!strict
-- Kemudi Studio mass-proxy setup utility.
--
-- Run this once from Roblox Studio's Command Bar after setup_vehicle_asset.luau:
--   1. Select exactly one prepared vehicle Model in Explorer.
--   2. Confirm it is under ServerStorage.KemudiVehicles.
--   3. Paste this file's contents into the Command Bar and run it.
--
-- This creates asset-side proxy geometry. It does not write a mass number,
-- set KemudiMassKg, or modify the generic runtime.

local Selection = game:GetService("Selection")
local ServerStorage = game:GetService("ServerStorage")

local selected = Selection:Get()
assert(#selected == 1, "Select exactly one prepared vehicle Model before running mass-proxy setup")

local vehicle = selected[1]
assert(vehicle:IsA("Model"), "The selected Instance must be a Model")
local vehiclesFolder = ServerStorage:FindFirstChild("KemudiVehicles")
assert(vehiclesFolder ~= nil and vehiclesFolder:IsA("Folder"), "Expected ServerStorage.KemudiVehicles")
assert(vehicle:IsDescendantOf(vehiclesFolder), "The vehicle must be under ServerStorage.KemudiVehicles")

local physicsRig = vehicle:FindFirstChild("PhysicsRig")
assert(physicsRig ~= nil and physicsRig:IsA("Folder"), "Missing PhysicsRig; run setup_vehicle_asset.luau first")
local render = vehicle:FindFirstChild("Render")
assert(render ~= nil and render:IsA("Folder"), "Missing Render; run setup_vehicle_asset.luau first")
local massProxies = physicsRig:FindFirstChild("MassProxies")
assert(massProxies ~= nil and massProxies:IsA("Folder"), "Missing PhysicsRig.MassProxies")

assert(
	massProxies:GetAttribute("GeneratedBy") == nil,
	"Mass proxies already exist; refusing to overwrite authored asset content"
)

local body = render:FindFirstChild("Pickup", true)
assert(body ~= nil and body:IsA("BasePart"), "Missing Render.Pickup body MeshPart")

local pivot = vehicle:GetPivot()
local bodySize = body.Size

-- Use the normalized body geometry to create four bounded proxy volumes. The
-- dimensions are asset geometry, not a mass calibration. Roblox derives the
-- resulting mass from each Part's material/physical properties at load time.
local proxySize = Vector3.new(
	math.max(bodySize.X * 0.45, 0.25),
	math.max(bodySize.Y * 0.45, 0.25),
	math.max(bodySize.Z * 0.22, 0.25)
)

local chassisNames = { "FL", "FR", "RL", "RR" }
local totalMass = 0

for _, wheelId in ipairs(chassisNames) do
	local chassis = physicsRig:FindFirstChild("Chassis_" .. wheelId)
	assert(chassis ~= nil and chassis:IsA("BasePart"), "Missing PhysicsRig.Chassis_" .. wheelId)

	local proxy = Instance.new("Part")
	proxy.Name = "MassProxy_" .. wheelId
	proxy.Size = proxySize
	proxy.CFrame = pivot * CFrame.new(pivot:PointToObjectSpace(chassis.Position))
	proxy.Anchored = true
	proxy.CanCollide = false
	proxy.CanTouch = false
	proxy.CanQuery = false
	proxy.Massless = false
	proxy.Material = Enum.Material.Metal
	proxy.Transparency = 0.7
	proxy.Color = Color3.fromRGB(70, 130, 255)
	proxy:SetAttribute("KemudiMassProxy", true)
	proxy:SetAttribute("KemudiNodeId", "Node_" .. wheelId)
	proxy:SetAttribute("KemudiMassSource", "roblox-physical-geometry")
	proxy.Parent = massProxies

	totalMass += proxy:GetMass()
end

massProxies:SetAttribute("GeneratedBy", "setup_vehicle_mass_proxies")
massProxies:SetAttribute("RequiresAuthoring", false)
massProxies:SetAttribute("MassSource", "roblox-physical-geometry")

local vehicleSpec = vehicle:FindFirstChild("VehicleSpec")
if vehicleSpec and vehicleSpec:IsA("Folder") then
	vehicleSpec:SetAttribute("MassPolicy", "proxy-geometry")
end

print(string.format(
	"Kemudi mass proxies prepared: %s; derived proxy mass %.2f Roblox mass units",
	massProxies:GetFullName(),
	totalMass
))
print("No vehicle mass constant or KemudiMassKg attribute was written.")
