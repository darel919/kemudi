-- Pure GroundZero chunk-grid helpers.
-- Kept separate from Roblox APIs so cell math can be tested with Lune.

local Grid = {}

function Grid.key(cellX, cellZ)
	return string.format("%d:%d", cellX, cellZ)
end

function Grid.cellForPosition(x, z, chunkSize)
	assert(type(x) == "number" and x == x, "x must be finite")
	assert(type(z) == "number" and z == z, "z must be finite")
	assert(type(chunkSize) == "number" and chunkSize > 0, "chunkSize must be positive")
	return math.floor(x / chunkSize), math.floor(z / chunkSize)
end

function Grid.center(cellX, cellZ, chunkSize, floorY)
	assert(type(chunkSize) == "number" and chunkSize > 0, "chunkSize must be positive")
	return (cellX + 0.5) * chunkSize, floorY or 0, (cellZ + 0.5) * chunkSize
end

function Grid.isWithin(cellX, cellZ, originX, originZ, radius)
	return math.max(math.abs(cellX - originX), math.abs(cellZ - originZ)) <= radius
end

function Grid.cellsAround(cellX, cellZ, radius)
	local cells = {}
	for z = cellZ - radius, cellZ + radius do
		for x = cellX - radius, cellX + radius do
			table.insert(cells, { x = x, z = z, key = Grid.key(x, z) })
		end
	end
	return cells
end

return Grid
