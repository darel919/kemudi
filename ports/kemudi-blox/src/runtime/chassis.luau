local Chassis = {}

local function normalize(x, y, z)
	local length = math.sqrt(x * x + y * y + z * z)
	if length <= 1e-8 or length ~= length then return nil end
	return x / length, y / length, z / length
end

local function dot(ax, ay, az, bx, by, bz)
	return ax * bx + ay * by + az * bz
end

local function average(nodes, indices)
	local x, y, z = 0, 0, 0
	for _, index in indices do
		local node = nodes[index]
		x += node.x
		y += node.y
		z += node.z
	end
	local count = #indices
	return x / count, y / count, z / count
end

-- Returns position and an orthonormal right/up/forward frame from the four
-- suspension mounts. The convention matches the physics chassis basis:
-- forward points from rear mounts to front mounts, right is passenger-side,
-- and up is right × forward.
function Chassis.frameFromNodes(nodes)
	if #nodes < 4 then return nil end
	local frontX, frontY, frontZ = average(nodes, { 1, 2 })
	local rearX, rearY, rearZ = average(nodes, { 3, 4 })
	local leftX, leftY, leftZ = average(nodes, { 1, 3 })
	local rightX, rightY, rightZ = average(nodes, { 2, 4 })
	local forwardX, forwardY, forwardZ = normalize(frontX - rearX, frontY - rearY, frontZ - rearZ)
	if not forwardX then return nil end

	local rawRightX, rawRightY, rawRightZ = rightX - leftX, rightY - leftY, rightZ - leftZ
	local projection = dot(rawRightX, rawRightY, rawRightZ, forwardX, forwardY, forwardZ)
	local frameRightX, frameRightY, frameRightZ = normalize(
		rawRightX - forwardX * projection,
		rawRightY - forwardY * projection,
		rawRightZ - forwardZ * projection)
	if not frameRightX then return nil end
	local upX, upY, upZ = normalize(
		frameRightY * forwardZ - frameRightZ * forwardY,
		frameRightZ * forwardX - frameRightX * forwardZ,
		frameRightX * forwardY - frameRightY * forwardX)
	if not upX then return nil end

	local centerX, centerY, centerZ
	if #nodes >= 8 then
		centerX, centerY, centerZ = average(nodes, { 5, 6, 7, 8 })
	else
		centerX, centerY, centerZ = average(nodes, { 1, 2, 3, 4 })
	end
	return {
		position = { centerX, centerY, centerZ },
		right = { frameRightX, frameRightY, frameRightZ },
		up = { upX, upY, upZ },
		forward = { forwardX, forwardY, forwardZ },
	}
end

return Chassis
