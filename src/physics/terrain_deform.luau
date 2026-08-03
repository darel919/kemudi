--[[
	Terrain deformation module — writes wheel contact deformations into
	the existing terrainRuts grid, and provides elastic recovery and
	rolling-resistance feedback to the vehicle system.

	ALL CONFIG MUST BE PROVIDED BY VEHICLE ASSET / MAP DATA.
	No vehicle-specific defaults in engine code.
--]]

local Module = {}

local function nan_guard(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

-- Install ----------------------------------------------------------------

function Module.install(World, Contract, Math)
	local gridSize = Contract.TERRAIN_GRID_SIZE
	local extent = Contract.TERRAIN_EXTENT
	local gridSizeSq = gridSize * gridSize

	-- Helper: safe grid index
	local function safeGridIndex(x, z)
		local gx = math.floor((x / extent + 0.5) * gridSize)
		local gz = math.floor((z / extent + 0.5) * gridSize)
		gx = clamp(gx, 0, gridSize - 1)
		gz = clamp(gz, 0, gridSize - 1)
		return gx, gz, gz * gridSize + gx + 1
	end

	-- State initialization — called once after World.new()
	-- ALL values MUST come from vehicle asset / map data
	function World:setupTerrainDeformation()
		-- Vehicle asset provides: tire contact patch area (m²)
		self.contact_patch_area = self.contact_patch_area
		-- Map data provides: surface hardness (0 = soft mud, 1 = concrete)
		self.surfaceHardness = self.surfaceHardness
		-- Map data provides: rut recovery rate (m/s)
		self.rutRecoveryRate = self.rutRecoveryRate
		-- Map data provides: maximum rut depth (m)
		self.maxRutDepth = self.maxRutDepth
		-- Vehicle asset provides: spread factor (0-1)
		self.spreadFactor = self.spreadFactor
		-- Map data provides: recovery depth damping (0-1)
		self.recoveryDepthDamping = self.recoveryDepthDamping
		-- Vehicle asset provides: max rolling resistance coefficient
		self.resistanceMax = self.resistanceMax
	end

	function World:_ensureTerrainDeformState()
		if self.maxRutDepth == nil then
			self:setupTerrainDeformation()
		end
	end

	-- Per-step deformation from grounded wheel contacts.
	function World:_updateTerrainDeformation(dt)
		self:_ensureTerrainDeformState()
		if not (dt > 0 and dt == dt) then return end

		local states = self.suspension_states
		if not states then return end

		local nodes = self.nodes
		local ruts = self.terrainRuts

		-- If required config is missing, skip deformation
		if not self.contact_patch_area or not self.surfaceHardness or not self.maxRutDepth then
			return
		end

		local contactArea = self.contact_patch_area
		local hardness = nan_guard(self.surfaceHardness, 0.5)
		local maxDepth = self.maxRutDepth
		local spreadFactor = nan_guard(self.spreadFactor, 0)

		-- Vehicle speed for speed factor
		local speed = 0
		if self.telemetry then
			speed = self.telemetry[Contract.T.SPEED_MPS + 1] or 0
		end
		if not (speed == speed) then speed = 0 end
		speed = math.max(speed, 0)

		-- Speed factor: slower = more deformation (more time in contact)
		local speedFactor = clamp(1 / (1 + speed * 0.5), 0.2, 1.0)

		for i = 1, math.min(#states, 4) do
			local st = states[i]
			if st and not st.is_airborne then
				local wheelLoad = nan_guard(st.load, 0)
				if wheelLoad > 0 then
					-- Contact point position
					local wx, wz
					if st.contact_point and st.contact_point[1] == st.contact_point[1] and st.contact_point[3] == st.contact_point[3] then
						wx = st.contact_point[1]
						wz = st.contact_point[3]
					else
						local n = nodes[i]
						if n then
							wx = n.x
							wz = n.z
						else
							continue
						end
					end

					-- Pressure-based deformation
					local pressure = wheelLoad / contactArea
					local hardnessFactor = 1 - clamp(hardness, 0, 1)
					local deformation = pressure * hardnessFactor * 0.00001 * dt * speedFactor

					if deformation > 0 then
						local gx, gz, idx = safeGridIndex(wx, wz)

						-- Write center cell
						local newVal = clamp((ruts[idx] or 0) + deformation, 0, maxDepth)
						ruts[idx] = newVal

						-- Spread to immediate neighbors (8-connected)
						for dz = -1, 1 do
							for dx = -1, 1 do
								if dx == 0 and dz == 0 then
									-- continue (Lua doesn't have continue, use if)
								else
									local nx_ = gx + dx
									local nz_ = gz + dz
									if nx_ >= 0 and nx_ < gridSize and nz_ >= 0 and nz_ < gridSize then
										local nIdx = nz_ * gridSize + nx_ + 1
										ruts[nIdx] = clamp(
											(ruts[nIdx] or 0) + deformation * spreadFactor,
											0, maxDepth
										)
									end
								end
							end
						end
					end
				end
			end
		end
	end

	-- Elastic recovery — call once per fixed step after deformation.
	function World:_recoverTerrainDeformation(dt)
		self:_ensureTerrainDeformState()
		if not (dt > 0 and dt == dt) then return end

		local ruts = self.terrainRuts
		local recoveryRate = nan_guard(self.rutRecoveryRate, 0)
		local maxDepth = self.maxRutDepth
		local recoveryDamping = nan_guard(self.recoveryDepthDamping, 0)

		if maxDepth <= 0 or recoveryRate <= 0 then return end

		for i = 1, gridSizeSq do
			local depth = ruts[i]
			if depth > 0 then
				-- Deeper ruts recover slower
				local depthRatio = depth / maxDepth
				local rate = recoveryRate * (1 - depthRatio * recoveryDamping)
				local newDepth = depth - rate * dt
				if newDepth < 0 then newDepth = 0 end
				ruts[i] = newDepth
			end
		end
	end

	-- API: set surface hardness (0 = soft mud, 1 = concrete/asphalt).
	function World:setSurfaceHardness(hardness)
		self:_ensureTerrainDeformState()
		self.surfaceHardness = clamp(hardness, 0, 1)
	end

	-- Query: rolling resistance from rut depth.
	-- Returns a coefficient in [0, resistanceMax].
	function World:getRutResistance(nodeX, nodeZ)
		self:_ensureTerrainDeformState()
		local _, _, idx = safeGridIndex(nodeX, nodeZ)
		local depth = self.terrainRuts[idx] or 0
		local maxDepth = self.maxRutDepth
		local resistanceMax = nan_guard(self.resistanceMax, 0)
		if maxDepth <= 0 or resistanceMax <= 0 then return 0 end
		return clamp(depth / maxDepth * resistanceMax, 0, resistanceMax)
	end

end -- Module.install

return Module