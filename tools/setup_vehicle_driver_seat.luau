--!strict
-- Kemudi Studio driver-seat authoring utility.
--
-- Run this once from Roblox Studio's Command Bar:
--   1. Select exactly one prepared vehicle Model in Explorer.
--   2. Confirm it is under ServerStorage.KemudiVehicles.
--   3. Paste this file into the Command Bar and run it.
--
-- The seat position is derived from the authored wheel/body geometry. This
-- utility creates content only; it does not set vehicle mass or physics tuning.

local Selection = game:GetService("Selection")
local ServerStorage = game:GetService("ServerStorage")

local selected = Selection:Get()
assert(#selected == 1, "Select exactly one prepared vehicle Model before running setup")

local vehicle = selected[1]
assert(vehicle:IsA("Model"), "The selected Instance must be a Model")

local vehiclesFolder = ServerStorage:FindFirstChild("KemudiVehicles")
assert(vehiclesFolder and vehiclesFolder:IsA("Folder"), "Expected ServerStorage.KemudiVehicles")
assert(vehicle:IsDescendantOf(vehiclesFolder), "The vehicle must be under ServerStorage.KemudiVehicles")

local render = vehicle:FindFirstChild("Render")
local spec = vehicle:FindFirstChild("VehicleSpec")
assert(render and render:IsA("Folder"), "Expected vehicle.Render Folder")
assert(spec and spec:IsA("Folder"), "Expected vehicle.VehicleSpec Folder")

local function findRenderPart(name: string): BasePart
	local instance = render:FindFirstChild(name, true)
	assert(instance and instance:IsA("BasePart"), "Missing render part: " .. name)
	return instance
end

local frontLeft = findRenderPart("FrontWheel_L")
local frontRight = findRenderPart("FrontWheel_R")
local rearAssembly = findRenderPart("BackWheels")
local body = findRenderPart("Pickup")

local existing = spec:FindFirstChild("DriverSeat")
if existing then
	assert(existing:IsA("Seat"), spec:GetFullName() .. ".DriverSeat must be a Seat")
	assert(existing:GetAttribute("KemudiDriverSeat") == true, "Existing DriverSeat is missing KemudiDriverSeat=true")
	print("Kemudi DriverSeat already authored: " .. existing:GetFullName())
	return
end

local pivot = vehicle:GetPivot()
local frontLeftLocal = pivot:PointToObjectSpace(frontLeft.Position)
local frontRightLocal = pivot:PointToObjectSpace(frontRight.Position)
local rearLocal = pivot:PointToObjectSpace(rearAssembly.Position)
local bodyLocal = pivot:PointToObjectSpace(body.Position)

local leftX = math.min(frontLeftLocal.X, frontRightLocal.X)
local rightX = math.max(frontLeftLocal.X, frontRightLocal.X)
local trackWidth = rightX - leftX
local frontZ = (frontLeftLocal.Z + frontRightLocal.Z) * 0.5
local seatZ = frontZ + (rearLocal.Z - frontZ) * 0.33
local seatX = leftX + trackWidth * 0.26
local seatY = bodyLocal.Y - body.Size.Y * 0.17

local seat = Instance.new("Seat")
seat.Name = "DriverSeat"
seat.Size = Vector3.new(math.min(trackWidth * 0.35, 2.2), 0.6, 1.8)
seat.CFrame = pivot * CFrame.new(seatX, seatY, seatZ)
seat.Anchored = true
seat.CanCollide = false
seat.CanTouch = true
seat.CanQuery = true
seat.Massless = true
seat.Transparency = 1
seat.CastShadow = false
seat:SetAttribute("KemudiDriverSeat", true)
seat:SetAttribute("KemudiDriverSeatVersion", 1)
seat.Parent = spec

print(string.format(
	"Kemudi DriverSeat authored at local frame (%.2f, %.2f, %.2f); derived from wheel/body geometry",
	seatX,
	seatY,
	seatZ
))
