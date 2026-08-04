--[[

	Terrain queries — surface properties, height lookup, rut depth, normal.
	Extracted from init.luau for maintainability.
	Installs methods on World via the standard subsystem pattern.

]]

local Module = {}

function Module.install(World, Contract, Math)

	-- Resolve the nearest authored road segment for wheel contact. Returning
	-- scalar properties avoids allocating a contact object in the fixed-step loop.
	function World:surfacePropertiesAt(x, z)
		local friction = self.groundFriction
		local roughness = self.surfaceRoughness
		local moisture = self.surfaceMoisture
		local compactness = self.surfaceCompactness
		local surfaceId = self.surfacePresetIndex
		local nearestDistance = math.huge
		for _, road in self.roadSurfaces do
			local points = road.points
			local halfWidth = road.width * 0.5
			for i = 1, #points - 1 do
				local ax, az = points[i][1], points[i][2]
				local bx, bz = points[i + 1][1], points[i + 1][2]
				local dx, dz = bx - ax, bz - az
				local lengthSquared = dx * dx + dz * dz
				local t = if lengthSquared > 1e-12 then math.clamp(((x - ax) * dx + (z - az) * dz) / lengthSquared, 0, 1) else 0
				local px, pz = ax + dx * t, az + dz * t
				local distanceSquared = (x - px) * (x - px) + (z - pz) * (z - pz)
				if distanceSquared <= halfWidth * halfWidth and distanceSquared < nearestDistance then
					nearestDistance = distanceSquared
					friction = road.friction
					roughness = road.roughness
					moisture = road.moisture
					compactness = road.compactness
					surfaceId = road.surfaceId
				end
			end
		end
		return friction, roughness, moisture, compactness, surfaceId
	end

	-- Terrain height query (profile fallback or heightmap).
	function World:terrainHeight(x, z)
		local baseHeight = 0
		if self.terrainSegments >= 1 and #self.terrainHeights > 0 then
			local seg = self.terrainSegments
			local gx = math.clamp((x / self.terrainWidth + 0.5) * seg, 0, seg)
			local gz = math.clamp((z / self.terrainDepth + 0.5) * seg, 0, seg)
			local ix = math.floor(gx)
			local iz = math.floor(gz)
			local x1 = math.min(ix + 1, seg)
			local z1 = math.min(iz + 1, seg)
			local fx = gx - ix
			local fz = gz - iz
			local stride = seg + 1
			local h00 = self.terrainHeights[iz * stride + ix + 1]
			local h10 = self.terrainHeights[iz * stride + x1 + 1]
			local h01 = self.terrainHeights[z1 * stride + ix + 1]
			local h11 = self.terrainHeights[z1 * stride + x1 + 1]
			baseHeight = (h00 + (h10 - h00) * fx) + ((h01 + (h11 - h01) * fx) - (h00 + (h10 - h00) * fx)) * fz
		else
			baseHeight = Math.terrainHeightForProfile(self.terrainProfile, x, z)
		end
		return baseHeight - self:rutDepthAt(x, z)
	end

	function World:rutDepthAt(x, z)
		local gridSize = Contract.TERRAIN_GRID_SIZE
		local extent = Contract.TERRAIN_EXTENT
		local gx = math.clamp(math.floor((x / extent + 0.5) * gridSize), 0, gridSize - 1)
		local gz = math.clamp(math.floor((z / extent + 0.5) * gridSize), 0, gridSize - 1)
		return self.terrainRuts[gz * gridSize + gx + 1]
	end

	function World:terrainNormal(x, z)
		local e = 0.25
		local hL = self:terrainHeight(x - e, z)
		local hR = self:terrainHeight(x + e, z)
		local hD = self:terrainHeight(x, z - e)
		local hU = self:terrainHeight(x, z + e)
		local nx, ny, nz = hL - hR, 2 * e, hD - hU
		local len = math.sqrt(nx * nx + ny * ny + nz * nz)
		if len < 1e-6 then len = 1e-6 end
		return { nx / len, ny / len, nz / len }
	end

end

return Module
