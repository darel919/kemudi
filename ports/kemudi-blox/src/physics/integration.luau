local Module = {}

function Module.install(World)
function World:_updateMassProperties()
	local nodes = self.nodes
	local count = #nodes
	local dynamicCount = 0
	for i = 1, count do
		if not nodes[i].fixed then dynamicCount += 1 end
	end
	if dynamicCount == 0 then return end
	-- Fuel mass follows the live tank level. Distribute the tank mass across
	-- dynamic nodes because the current public vehicle schema does not carry a
	-- dedicated tank position; this still preserves total weight and inertia as
	-- fuel burns or is refilled.
	local fuelMass = 0
	if self.fuel_tank then
		local level = self.fuel_tank.current_level
		local density = self.fuel_tank.fuel_density
		if type(level) == "number" and level == level and type(density) == "number" and density == density then
			fuelMass = math.max(level * density, 0)
		end
	end
	local fuelShare = fuelMass / dynamicCount
 	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local baseMass = self.baseNodeMasses[i] or n.mass
			if baseMass < 1e-6 then baseMass = 1e-6 end
			n.mass = baseMass + fuelShare
			n.invMass = 1 / n.mass
		end
	end
end

function World:_applyForces()
	local nodes = self.nodes
	local count = #nodes
	-- Compute total mass
	local totalMass = 0
	for i = 1, count do
		if not nodes[i].fixed then
			totalMass += math.max(nodes[i].mass, 0)
		end
	end
	if totalMass <= 0 then return end

	-- Average velocity
	local avgVx, avgVy, avgVz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local m = math.max(n.mass, 0)
			avgVx += n.vx * m / totalMass
			avgVy += n.vy * m / totalMass
			avgVz += n.vz * m / totalMass
		end
	end
	local speed = math.sqrt(avgVx * avgVx + avgVy * avgVy + avgVz * avgVz)
	-- Downforce (simplified — same formula as Rust body_downforce_force)
	local safeSpeed = math.abs(speed)
	local rho = math.max(self.airDensity, 0)
	local area = math.max(self.frontalArea, 0)
	local drag = math.abs(self.dragCoefficient)
	local downforceCoeff = math.clamp(drag * 0.25, 0, 1)
	local downforce = math.max(0.5 * rho * area * downforceCoeff * safeSpeed * safeSpeed, 0)

	-- Apply gravity and downforce
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			n.fy += n.mass * self.gravity
			n.fy -= downforce * math.max(n.mass, 0) / totalMass
		end
	end
end

function World:_integrateVelocities(dt: number)
	local nodes = self.nodes
	local count = #nodes
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			n.vx += n.fx * n.invMass * dt
			n.vy += n.fy * n.invMass * dt
			n.vz += n.fz * n.invMass * dt
			n.lastForce = math.sqrt(n.fx * n.fx + n.fy * n.fy + n.fz * n.fz)
			n.fx = 0
			n.fy = 0
			n.fz = 0
			n.vx = Math.finite_or_zero(math.clamp(n.vx, -200, 200))
			n.vy = Math.finite_or_zero(math.clamp(n.vy, -200, 200))
			n.vz = Math.finite_or_zero(math.clamp(n.vz, -200, 200))
			n.x = Math.finite_or_zero(n.x + n.vx * dt)
			n.y = Math.finite_or_zero(n.y + n.vy * dt)
			n.z = Math.finite_or_zero(n.z + n.vz * dt)
		end
	end
end
end

return Module
