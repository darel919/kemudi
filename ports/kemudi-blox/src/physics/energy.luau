--[[
	Energy conservation tracking for the physics simulation.
	Tracks kinetic, potential, thermal, and deformation energy per frame.
	Reports energy drift for validation.
--]]

local Module = {}

local function nan_guard(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

function Module.install(World, Math)

	-- State initialization
	function World:setupEnergyTracking()
		-- Gravitational constant
		self.gravity = self.gravity or 9.81
	end

	-- Called each step to compute and track energy
	function World:_computeEnergy()
		local nodes = self.nodes
		if not nodes or #nodes == 0 then return end

		self.energy = self.energy or {
			kinetic = 0,
			potential = 0,
			thermal = 0,
			deformation = 0,
			total = 0,
			previousTotal = 0,
			drift = 0,
		}

		local g = self.gravity
		local kin = 0
		local pot = 0
		local def = 0

		-- Kinetic + potential energy from nodes
		for i = 1, #nodes do
			local n = nodes[i]
			if not n.fixed then
				local m = math.max(n.mass, 0)
				local v2 = n.vx*n.vx + n.vy*n.vy + n.vz*n.vz
				kin = kin + 0.5 * m * v2
				pot = pot + m * g * math.max(n.y, 0)
			end
		end

		-- Deformation energy from beams (0.5 * k * (len - rest)^2)
		local beams = self.beams
		if beams then
			for i = 1, #beams do
				local b = beams[i]
				if not b.broken then
					local a = self.nodes[b.nodeA + 1]
					local c = self.nodes[b.nodeB + 1]
					if a and c then
						local dx = c.x - a.x
						local dy = c.y - a.y
						local dz = c.z - a.z
						local len = math.sqrt(dx*dx + dy*dy + dz*dz)
						local stretch = len - b.length
						if stretch ~= stretch then stretch = 0 end
						local k = b.stiffness or 0
						if k > 0 then
							def = def + 0.5 * k * stretch * stretch
						end
					end
				end
			end
		end

		-- Thermal energy from subsystems (approximate)
		local therm = 0
		if self.engine_thermal then
			local et = self.engine_thermal
			if et.thermal and et.thermal.coolant_temp then
				-- Convert coolant temp to energy (approximate)
				therm = therm + (et.thermal.coolant_temp - 20) * 100000 -- arbitrary scaling
			end
		end
		if self.tire_thermal then
			for i = 1, #self.tire_thermal do
				local tt = self.tire_thermal[i]
				if tt and tt.temperature then
					therm = therm + (tt.temperature - 20) * 50000
				end
			end
		end

		-- Total energy
		local total = kin + pot + therm + def

		-- Drift
		local prev = self.energy.previousTotal or total
		local drift = total - prev

		-- Store
		self.energy.kinetic = kin
		self.energy.potential = pot
		self.energy.thermal = therm
		self.energy.deformation = def
		self.energy.total = total
		self.energy.previousTotal = total
		self.energy.drift = drift
	end

	-- Get energy report for telemetry
	function World:getEnergyReport()
		if not self.energy then return {} end
		return {
			kinetic = self.energy.kinetic,
			potential = self.energy.potential,
			thermal = self.energy.thermal,
			deformation = self.energy.deformation,
			total = self.energy.total,
			drift = self.energy.drift,
		}
	end

end -- Module.install

return Module