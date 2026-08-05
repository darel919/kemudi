--[[

	World geometry management — nodes, beams, triangles, collision primitives,
	road surfaces, auto-triangle generation, volume computation, controls.
	Extracted from init.luau for maintainability.
	Installs methods on World via the standard subsystem pattern.

]]

local Module = {}

function Module.install(World, Math, Contract)

	-- Add a node, matching Rust PhysicsWorld::add_node.
	function World:addNode(id, x, y, z, mass, fixed, staticCollision)
		x = Math.finite_or_zero(x)
		y = Math.finite_or_zero(y)
		z = Math.finite_or_zero(z)
		if not (mass > 0 and mass == mass) then mass = 1 end -- NaN/guard
		local invMass = if fixed then 0 else 1 / mass
		local nodeCount = #self.nodes
		table.insert(self.nodes, {
			id = id,
			x = x, y = y, z = z,
			vx = 0, vy = 0, vz = 0,
			mass = mass,
			invMass = invMass,
			fixed = fixed,
			-- Suspension nodes are excluded from analytic terrain collision because
			-- suspension resolves their ground contact, but every dynamic node must
			-- still be eligible for authored static map geometry.
			collision = (not fixed) and nodeCount >= 4,
			staticCollision = if staticCollision == nil then not fixed else (not fixed) and staticCollision == true,
			fx = 0, fy = 0, fz = 0,
			lastForce = 0,
		})
		table.insert(self.constraintStartPositions, {x, y, z})
		table.insert(self.restPositions, {x, y, z})
		table.insert(self.baseNodeMasses, mass)
	end

	function World:setNodeCollision(nodeId, collision)
		local node = self.nodes[nodeId + 1] -- 0-indexed externally
		if node then
			node.collision = (nodeId >= 4) and (not node.fixed) and collision
			node.staticCollision = (not node.fixed) and collision
		end
	end

	-- Add a beam between two node indices (0-based).
	function World:addBeam(id, nodeA, nodeB, stiffness, damping, strength)
		local a = self.nodes[nodeA + 1]
		local b = self.nodes[nodeB + 1]
		local length = 1
		if a and b then
			length = Math.distance3(
				{a.x, a.y, a.z}, {b.x, b.y, b.z}
			)
			if length < 1e-4 then length = 1e-4 end
		end
		if stiffness < 1 then stiffness = 1 end
		if damping < 0 then damping = 0 end
		if strength < 0 then strength = 0 end
		table.insert(self.beams, {
			id = id,
			nodeA = nodeA,
			nodeB = nodeB,
			stiffness = stiffness,
			damping = damping,
			strength = strength,
			yieldStrength = strength * 0.55,
			plasticity = 0.25,
			length = length,
			initialLength = length,
			broken = false,
			lambda = 0,
			stress = 0,
			-- Beam type: "normal" (default), "pressured"
			type = "normal",
			pressure = 0,
			restPressure = 0,
			minRatio = 0.5,
			maxRatio = 1.5,
		})
	end

	function World:setBeamMaterial(beamId, yieldStrength, plasticity)
		for _, beam in self.beams do
			if beam.id == beamId then
				if yieldStrength == yieldStrength then -- finite check
					beam.yieldStrength = math.clamp(yieldStrength, 0, beam.strength)
				end
				if plasticity == plasticity then
					beam.plasticity = math.clamp(plasticity, 0, 1)
				end
				break
			end
		end
	end

	-- Add a pressurized beam whose rest length varies with internal pressure.
	-- Used for tire sidewalls: low pressure → compressed (squishy), high pressure → extended (stiff).
	function World:addPressuredBeam(id, nodeA, nodeB, stiffness, damping, strength, restPressure, minRatio, maxRatio)
		local beamIndex = nil
		for i, beam in self.beams do
			if beam.id == id then beamIndex = i; break end
		end
		if not beamIndex then
			-- Create the beam first, then mark it as pressured
			self:addBeam(id, nodeA, nodeB, stiffness, damping, strength)
			beamIndex = #self.beams
		end
		local beam = self.beams[beamIndex]
		beam.type = "pressured"
		beam.restPressure = restPressure or 220
		beam.pressure = beam.restPressure
		beam.minRatio = minRatio or 0.5
		beam.maxRatio = maxRatio or 1.5
	end

	-- Set the internal pressure of a beam (e.g., tire inflation).
	function World:setBeamPressure(beamId, pressure)
		for _, beam in self.beams do
			if beam.id == beamId then
				beam.pressure = math.max(pressure or 0, 0)
				break
			end
		end
	end

	-- Add a triangle area-preservation constraint.
	function World:addTriangle(a, b, c)
		local na = self.nodes[a + 1]
		local nb = self.nodes[b + 1]
		local nc = self.nodes[c + 1]
		if not (na and nb and nc) then return end
		local area = Math.triangleArea(
			na.x, na.y, na.z,
			nb.x, nb.y, nb.z,
			nc.x, nc.y, nc.z
		)
		if area > 1e-8 then
			table.insert(self.triangles, { a = a, b = b, c = c, restArea = area, lambda = 0 })
		end
	end

	function World:addCollisionBox(cx, cy, cz, hx, hy, hz, restitution, friction, right, up, forward)
		local basis = nil
		if type(right) == "table" and type(up) == "table" and type(forward) == "table" then
			basis = { right, up, forward }
		end
		table.insert(self.collisionBoxes, {
			center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
			halfExtents = {
				math.max(math.abs(hx), 1e-4),
				math.max(math.abs(hy), 1e-4),
				math.max(math.abs(hz), 1e-4),
			},
			restitution = math.clamp(restitution, 0, 1),
			friction = math.clamp(friction, 0, 1),
			basis = basis,
		})
	end

	function World:addCollisionSphere(cx, cy, cz, radius, restitution, friction)
		table.insert(self.collisionSpheres, {
			center = { Math.finite_or_zero(cx), Math.finite_or_zero(cy), Math.finite_or_zero(cz) },
			radius = math.max(math.abs(radius), 1e-4),
			restitution = math.clamp(restitution, 0, 1),
			friction = math.clamp(friction, 0, 1),
		})
	end

	function World:addBoundary(x, y, z, nx, ny, nz, restitution)
		local length = math.sqrt(nx * nx + ny * ny + nz * nz)
		if length < 1e-8 or length ~= length then
			nx, ny, nz = 0, 1, 0
		else
			nx, ny, nz = nx / length, ny / length, nz / length
		end
		table.insert(self.boundaries, {
			point = { Math.finite_or_zero(x), Math.finite_or_zero(y), Math.finite_or_zero(z) },
			normal = { nx, ny, nz },
			restitution = math.clamp(restitution, 0, 1),
			friction = 0.8,
		})
	end

	function World:addRoadSurface(points, width, surfaceId, friction, roughness, moisture, compactness)
		local polyline = {}
		for i = 1, #points, 2 do
			local px, pz = points[i], points[i + 1]
			if px == px and pz == pz then -- finite check
				table.insert(polyline, { px, pz })
			end
		end
		if #polyline < 2 then return end
		table.insert(self.roadSurfaces, {
			points = polyline,
			width = math.clamp(math.abs(width), 0.01, 10000),
			surfaceId = surfaceId,
			friction = math.clamp(friction, 0, 2),
			roughness = math.clamp(roughness, 0, 1),
			moisture = math.clamp(moisture, 0, 1),
			compactness = math.clamp(compactness, 0, 1),
		})
	end

	-- Auto-generate triangles from the beam graph (finds all 3-cycles).
	-- Call after all beams and nodes are added, before computeRestVolume.
	function World:autoGenerateTriangles()
		local adj = {}
		for i = 1, #self.nodes do adj[i] = {} end
		for _, beam in self.beams do
			if not beam.broken then
				local a, b = beam.nodeA + 1, beam.nodeB + 1
				if a <= #self.nodes and b <= #self.nodes then
					adj[a][b] = true
					adj[b][a] = true
				end
			end
		end
		for i = 1, #self.nodes do
			for j in adj[i] do
				if j > i then
					for k in adj[j] do
						if k > j and adj[i][k] then
							self:addTriangle(i - 1, j - 1, k - 1)
						end
					end
				end
			end
		end
	end

	-- Compute the rest volume and centroid of the triangle mesh.
	-- Call once after autoGenerateTriangles or manual triangle setup.
	function World:computeRestVolume()
		local triangles = self.triangles
		local nodes = self.nodes
		if #triangles == 0 then return end

		-- Centroid of all dynamic nodes
		local ccx, ccy, czz, count = 0, 0, 0, 0
		for i = 1, #nodes do
			if not nodes[i].fixed then
				ccx += nodes[i].x
				ccy += nodes[i].y
				czz += nodes[i].z
				count += 1
			end
		end
		if count > 0 then ccx, ccy, czz = ccx / count, ccy / count, czz / count end
		self.restCentroid = { ccx, ccy, czz }

		-- Signed volume from triangle fan around centroid
		local volume = 0
		for ti = 1, #triangles do
			local tri = triangles[ti]
			local a = nodes[tri.a + 1]
			local b = nodes[tri.b + 1]
			local c = nodes[tri.c + 1]
			if a and b and c then
				local ax, ay, az = a.x - ccx, a.y - ccy, a.z - czz
				local bx, by, bz = b.x - ccx, b.y - ccy, b.z - czz
				local cx2, cy2, cz2 = c.x - ccx, c.y - ccy, c.z - czz
				volume += (ax * (by * cz2 - bz * cy2) + ay * (bz * cx2 - bx * cz2) + az * (bx * cy2 - by * cx2)) / 6
			end
		end
		self.restVolume = volume
	end

	function World:setDrivetrainLayout(layout)
		if type(layout) == "string" then
			layout = ({ RWD = 0, FWD = 1, AWD = 2, rwd = 0, fwd = 1, awd = 2 })[layout] or 0
		end
		if type(layout) ~= "number" or layout ~= layout then layout = 0 end
		self.driveLayout = math.clamp(math.floor(layout), 0, 2)
	end

	function World:setControls(controls)
		controls = controls or {}
		local function bounded(value, low, high)
			if type(value) ~= "number" or value ~= value then value = 0 end
			return math.clamp(value, low, high)
		end
		self.controls.steering = bounded(controls.steering, -1, 1)
		self.controls.throttle = bounded(controls.throttle, 0, 1)
		self.controls.brake = bounded(controls.brake, 0, 1)
		self.controls.clutch = bounded(controls.clutch, 0, 1)
		self.controls.handbrake = controls.handbrake == true
		self.controls.gear_up = controls.gear_up == true
		self.controls.gear_down = controls.gear_down == true
		self.controls.selector_up = controls.selector_up == true
		self.controls.selector_down = controls.selector_down == true
		self.controls.manual_shift_up = controls.manual_shift_up == true
		self.controls.manual_shift_down = controls.manual_shift_down == true
		self.controls.engine_on = controls.engine_on == true
		local stage = if type(controls.engine_stage) == "number" then controls.engine_stage else 0
		self.controls.engine_stage = math.clamp(math.floor(stage), 0, 2)
	end

end

return Module
