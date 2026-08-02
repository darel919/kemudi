local Module = {}

function Module.install(World)
-- Aerodynamic drag
function World:_applyDrag()
	local nodes = self.nodes
	local count = #nodes
	local totalMass = 0
	for i = 1, count do
		if not nodes[i].fixed then totalMass += nodes[i].mass end
	end
	totalMass = math.max(totalMass, 1)
	local avgVx, avgVy, avgVz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			avgVx += n.vx * n.mass / totalMass
			avgVy += n.vy * n.mass / totalMass
			avgVz += n.vz * n.mass / totalMass
		end
	end
	local speed = math.sqrt(avgVx*avgVx + avgVy*avgVy + avgVz*avgVz)
	local totalDrag = 0.5 * self.airDensity * self.dragCoefficient * self.frontalArea * speed * speed
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local ns = math.sqrt(n.vx*n.vx + n.vy*n.vy + n.vz*n.vz)
			if ns > 1e-6 then
				local drag = totalDrag * n.mass / totalMass
				local factor = math.max(1 - drag * self.fixedDt / (n.mass * ns + 1e-6), 0)
				n.vx *= factor
				n.vy *= factor
				n.vz *= factor
			end
		end
	end
end
end

return Module
