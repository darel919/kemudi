--!strict
-- Kemudi Studio vehicle asset setup utility.
--
-- Run this once from Roblox Studio's Command Bar:
--   1. Select exactly one imported vehicle Model in Explorer.
--   2. Confirm it is under ServerStorage.KemudiVehicles.
--   3. Paste this file into the Command Bar and run it.
--
-- This utility only prepares content. It does not spawn a vehicle, author
-- vehicle mass, or start the physics runtime.

local Selection = game:GetService("Selection")

local selected = Selection:Get()
assert(#selected == 1, "Select exactly one imported vehicle Model before running setup")

local vehicle = selected[1]
assert(vehicle:IsA("Model"), "The selected Instance must be a Model")
assert(vehicle:IsDescendantOf(game:GetService("ServerStorage")), "The vehicle must be under ServerStorage")

local vehiclesFolder = game:GetService("ServerStorage"):FindFirstChild("KemudiVehicles")
assert(vehiclesFolder ~= nil and vehiclesFolder:IsA("Folder"), "Expected ServerStorage.KemudiVehicles")
assert(vehicle:IsDescendantOf(vehiclesFolder), "The vehicle must be under ServerStorage.KemudiVehicles")

local _, importedSize = vehicle:GetBoundingBox()
local largestImportedDimension = math.max(importedSize.X, importedSize.Y, importedSize.Z)
assert(
	largestImportedDimension > 100,
	"The selected vehicle appears already scaled; refusing to apply the import scale twice"
)

-- The downloaded FBX was authored in centimetres. Kemudi's simulation uses
-- metres, and Roblox's conventional scale is approximately 1 stud = 0.28 m.
-- This is a generic source-unit conversion, not vehicle-specific tuning.
local SOURCE_CENTIMETERS_TO_STUDS = 1 / 28
vehicle:ScaleTo(vehicle:GetScale() * SOURCE_CENTIMETERS_TO_STUDS)

local function getOrCreateFolder(parent: Instance, name: string): Folder
	local existing = parent:FindFirstChild(name)
	if existing ~= nil then
		assert(existing:IsA("Folder"), parent:GetFullName() .. "." .. name .. " must be a Folder")
		return existing
	end

	local folder = Instance.new("Folder")
	folder.Name = name
	folder.Parent = parent
	return folder
end

local render = getOrCreateFolder(vehicle, "Render")
local physicsRig = getOrCreateFolder(vehicle, "PhysicsRig")
local wheels = getOrCreateFolder(vehicle, "Wheels")
local vehicleSpec = getOrCreateFolder(vehicle, "VehicleSpec")
local massProxies = getOrCreateFolder(physicsRig, "MassProxies")

-- The importer places the render MeshParts directly under the Model. Move only
-- those direct render children; existing authored folders are preserved.
for _, child in ipairs(vehicle:GetChildren()) do
	if child:IsA("BasePart") then
		child.Parent = render
	end
end

for _, descendant in ipairs(render:GetDescendants()) do
	if descendant:IsA("BasePart") then
		descendant.Anchored = true
		descendant.CanCollide = false
		descendant.CanTouch = false
		descendant.CanQuery = false
		descendant.Massless = true
	end
end

local function findRenderPart(name: string): BasePart
	local instance = render:FindFirstChild(name, true)
	assert(instance ~= nil and instance:IsA("BasePart"), "Missing render wheel/body part: " .. name)
	return instance
end

local frontLeft = findRenderPart("FrontWheel_L")
local frontRight = findRenderPart("FrontWheel_R")
local rearAssembly = findRenderPart("BackWheels")
local body = findRenderPart("Pickup")

local pivot = vehicle:GetPivot()
local frontLeftLocal = pivot:PointToObjectSpace(frontLeft.Position)
local frontRightLocal = pivot:PointToObjectSpace(frontRight.Position)
local rearAssemblyLocal = pivot:PointToObjectSpace(rearAssembly.Position)
local bodyLocal = pivot:PointToObjectSpace(body.Position)

local leftX = math.min(frontLeftLocal.X, frontRightLocal.X)
local rightX = math.max(frontLeftLocal.X, frontRightLocal.X)
local trackHalf = (rightX - leftX) * 0.5
local rearLeftLocal = Vector3.new(rearAssemblyLocal.X - trackHalf, rearAssemblyLocal.Y, rearAssemblyLocal.Z)
local rearRightLocal = Vector3.new(rearAssemblyLocal.X + trackHalf, rearAssemblyLocal.Y, rearAssemblyLocal.Z)

local function upsertMarker(name: string, localPosition: Vector3, role: string): Part
	local existing = physicsRig:FindFirstChild(name)
	if existing ~= nil then
		assert(existing:IsA("Part"), physicsRig:GetFullName() .. "." .. name .. " must be a Part")
	end

	local marker = if existing then existing else Instance.new("Part")
	marker.Name = name
	marker.Size = Vector3.new(0.15, 0.15, 0.15)
	marker.CFrame = pivot * CFrame.new(localPosition)
	marker.Anchored = true
	marker.CanCollide = false
	marker.CanTouch = false
	marker.CanQuery = false
	marker.Massless = true
	marker.Transparency = 0.35
	marker.Color = Color3.fromRGB(255, 170, 0)
	marker:SetAttribute("KemudiNodeRole", role)
	marker.Parent = physicsRig
	return marker
end

-- Wheel nodes are positioned from imported wheel geometry. Chassis markers are
-- intentionally separate so later authoring can move them without touching the
-- visible mesh. No mass is assigned here.
upsertMarker("Node_FL", frontLeftLocal, "wheel-front-left")
upsertMarker("Node_FR", frontRightLocal, "wheel-front-right")
upsertMarker("Node_RL", rearLeftLocal, "wheel-rear-left")
upsertMarker("Node_RR", rearRightLocal, "wheel-rear-right")

local chassisY = bodyLocal.Y
upsertMarker("Chassis_FL", Vector3.new(leftX, chassisY, frontLeftLocal.Z), "chassis-front-left")
upsertMarker("Chassis_FR", Vector3.new(rightX, chassisY, frontRightLocal.Z), "chassis-front-right")
upsertMarker("Chassis_RL", Vector3.new(leftX, chassisY, rearAssemblyLocal.Z), "chassis-rear-left")
upsertMarker("Chassis_RR", Vector3.new(rightX, chassisY, rearAssemblyLocal.Z), "chassis-rear-right")

local function upsertWheel(name: string, visual: BasePart, role: string, steered: boolean)
	local wheel = wheels:FindFirstChild(name)
	if wheel == nil then
		wheel = Instance.new("Folder")
		wheel.Name = name
		wheel.Parent = wheels
	end
	assert(wheel:IsA("Folder"), wheels:GetFullName() .. "." .. name .. " must be a Folder")

	local visualRef = wheel:FindFirstChild("Visual")
	if visualRef == nil then
		visualRef = Instance.new("ObjectValue")
		visualRef.Name = "Visual"
		visualRef.Parent = wheel
	end
	assert(visualRef:IsA("ObjectValue"), wheel:GetFullName() .. ".Visual must be an ObjectValue")
	visualRef.Value = visual

	wheel:SetAttribute("KemudiWheelRole", role)
	wheel:SetAttribute("Steered", steered)
	wheel:SetAttribute("RadiusStuds", math.max(visual.Size.Y, visual.Size.Z) * 0.5)
end

upsertWheel("FL", frontLeft, "front-left", true)
upsertWheel("FR", frontRight, "front-right", true)
-- The imported asset contains one combined rear-wheel MeshPart. Both anchors
-- reference it until separate rear visual meshes are authored.
upsertWheel("RL", rearAssembly, "rear-left", false)
upsertWheel("RR", rearAssembly, "rear-right", false)

vehicle:SetAttribute("KemudiVehicleAssetVersion", 1)
vehicle:SetAttribute("KemudiSourceUnit", "centimeter")
vehicle:SetAttribute("KemudiSourceScaleApplied", SOURCE_CENTIMETERS_TO_STUDS)
vehicleSpec:SetAttribute("SchemaVersion", 1)
vehicleSpec:SetAttribute("SimulationLengthUnit", "meter")
vehicleSpec:SetAttribute("RobloxLengthUnit", "stud")
vehicleSpec:SetAttribute("StudsPerMeter", 1 / 0.28)
massProxies:SetAttribute("RequiresAuthoring", true)

local _, normalizedSize = vehicle:GetBoundingBox()
print(string.format(
	"Kemudi vehicle prepared: %s; normalized bounds %.2f x %.2f x %.2f studs",
	vehicle:GetFullName(),
	normalizedSize.X,
	normalizedSize.Y,
	normalizedSize.Z
))
print("MassProxy authoring is still required; no vehicle mass was assigned by this utility.")
