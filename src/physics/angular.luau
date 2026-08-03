--[[
	Angular dynamics — 6-DOF rotation, diagonal inertia tensor,
	and rigid chassis shape matching for soft-body vehicles.

	Called in this order during stepFixed:
	  1. _computeInertiaTensor()   (every N steps for perf)
	  2. [forces applied]          (_applyForces accumulates n.fx/fy/fz)
	  3. _computeAngularVelocity() (reads n.fx/fy/fz for torque — must be AFTER forces)
	  4. _applyAngularMotion()
	  5. [XPBD solve]
	  6. _shapeMatchingProjection()

	ALL VEHICLE-SPECIFIC PARAMETERS MUST COME FROM VEHICLE ASSET.
	Engine code provides NO defaults for inertia, damping, shape matching stiffness, etc.
--]]

local Module = {}

local function nan_guard(v, fallback)
	if v == nil then return fallback end
	if v ~= v then return fallback end -- NaN
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

function Module.install(World, Math)

-- Compute center of mass, diagonal inertia tensor, and chassis orientation vectors.
-- Rest-relative positions are cached once (first call) for shape matching.
function World:_computeInertiaTensor()
	local nodes = self.nodes
	local count = #nodes

	-- === Center of mass (weighted by mass, dynamic nodes only) ===
	local totalMass = 0
	local cx, cy, cz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local m = math.max(n.mass, 0)
			totalMass += m
			cx += n.x * m
			cy += n.y * m
			cz += n.z * m
		end
	end
	if totalMass <= 1e-8 then return end
	local invMass = 1 / totalMass
	cx = nan_guard(cx * invMass, 0)
	cy = nan_guard(cy * invMass, 0)
	cz = nan_guard(cz * invMass, 0)
	self.centerOfMass = { cx, cy, cz }

	-- === Diagonal inertia tensor ===
	-- Ixx = Σ m*(dy²+dz²), Iyy = Σ m*(dx²+dz²), Izz = Σ m*(dx²+dy²)
	local ix, iy, iz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local m = math.max(n.mass, 0)
			local dx = n.x - cx
			local dy = n.y - cy
			local dz = n.z - cz
			ix += m * (dy * dy + dz * dz)
			iy += m * (dx * dx + dz * dz)
			iz += m * (dx * dx + dy * dy)
		end
	end
	-- Floor to prevent division by zero in angular velocity integration
	-- If vehicle asset doesn't provide override, compute from node masses/positions
	self.inertia = {
		math.max(nan_guard(ix, 100), 1e-3),
		math.max(nan_guard(iy, 100), 1e-3),
		math.max(nan_guard(iz, 100), 1e-3),
	}

	-- Vehicle asset may override inertia with explicit values (Ixx, Iyy, Izz in kg·m²)
	if self.inertia_override then
		self.inertia = self.inertia_override
	end

	-- === Forward / right vectors from suspension nodes 1-4 ===
	-- Front = avg(1,2), Rear = avg(3,4), Left = avg(1,3), Right = avg(2,4)
	-- Vehicle asset may provide custom chassis node indices via self.chassis_nodes
	if count >= 4 then
		local front_idx = (self.chassis_nodes and self.chassis_nodes.front) or {1, 2}
		local rear_idx = (self.chassis_nodes and self.chassis_nodes.rear) or {3, 4}
		local left_idx = (self.chassis_nodes and self.chassis_nodes.left) or {1, 3}
		local right_idx = (self.chassis_nodes and self.chassis_nodes.right) or {2, 4}

		local fX = (nodes[front_idx[1]].x + nodes[front_idx[2]].x) * 0.5
		local fY = (nodes[front_idx[1]].y + nodes[front_idx[2]].y) * 0.5
		local fZ = (nodes[front_idx[1]].z + nodes[front_idx[2]].z) * 0.5
		local rX = (nodes[rear_idx[1]].x + nodes[rear_idx[2]].x) * 0.5
		local rY = (nodes[rear_idx[1]].y + nodes[rear_idx[2]].y) * 0.5
		local rZ = (nodes[rear_idx[1]].z + nodes[rear_idx[2]].z) * 0.5
		local lX = (nodes[left_idx[1]].x + nodes[left_idx[2]].x) * 0.5
		local lY = (nodes[left_idx[1]].y + nodes[left_idx[2]].y) * 0.5
		local lZ = (nodes[left_idx[1]].z + nodes[left_idx[2]].z) * 0.5
		local rtX = (nodes[right_idx[1]].x + nodes[right_idx[2]].x) * 0.5
		local rtY = (nodes[right_idx[1]].y + nodes[right_idx[2]].y) * 0.5
		local rtZ = (nodes[right_idx[1]].z + nodes[right_idx[2]].z) * 0.5

		local fx, fy, fz = fX - rX, fY - rY, fZ - rZ
		local flen = math.sqrt(fx*fx + fy*fy + fz*fz)
		if flen > 1e-8 then
			fx, fy, fz = fx/flen, fy/flen, fz/flen
			self.chassisForward = { fx, fy, fz }
		end

		local rx, ry, rz = rtX - lX, rtY - lY, rtZ - lZ
		-- Project right onto plane perpendicular to forward
		local proj = rx*fx + ry*fy + rz*fz
		rx, ry, rz = rx - fx*proj, ry - fy*proj, rz - fz*proj
		local rlen = math.sqrt(rx*rx + ry*ry + rz*rz)
		if rlen > 1e-8 then
			rx, ry, rz = rx/rlen, ry/rlen, rz/rlen
			self.chassisRight = { rx, ry, rz }
		end

		-- Up = right × forward (matches the chassis frame convention).
		local ux = ry*fz - rz*fy
		local uy = rz*fx - rx*fz
		local uz = rx*fy - ry*fx
		local ulen = math.sqrt(ux*ux + uy*uy + uz*uz)
		if ulen > 1e-8 then
			ux, uy, uz = ux/ulen, uy/ulen, uz/ulen
			self.chassisUp = { ux, uy, uz }
		end
	end

	-- === Cache rest positions for shape matching ===
	if #self.restPositions == 0 then
		for i = 1, count do
			local n = nodes[i]
			if not n.fixed then
				self.restPositions[i] = { n.x, n.y, n.z }
			end
		end
	end

	local rm, rcx, rcy, rcz = 0, 0, 0, 0
	for i = 1, count do
		local rp = self.restPositions[i]
		if rp and not nodes[i].fixed then
			local m = math.max(nodes[i].mass, 0)
			rm += m
			rcx += rp[1] * m
			rcy += rp[2] * m
			rcz += rp[3] * m
		end
	end
	if rm > 1e-8 then
		rcx /= rm
		rcy /= rm
		rcz /= rm
	end
	self.restCoM = { nan_guard(rcx, 0), nan_guard(rcy, 0), nan_guard(rcz, 0) }

	if #self.restRelativePositions == 0 then
		local restRight = self.chassisRight or { 1, 0, 0 }
		local restUp = self.chassisUp or { 0, 1, 0 }
		local restForward = self.chassisForward or { 0, 0, -1 }
		for i = 1, count do
			local rp = self.restPositions[i]
			if rp and not nodes[i].fixed then
				local dx = rp[1] - self.restCoM[1]
				local dy = rp[2] - self.restCoM[2]
				local dz = rp[3] - self.restCoM[3]
				self.restRelativePositions[i] = {
					dx * restRight[1] + dy * restRight[2] + dz * restRight[3],
					dx * restUp[1] + dy * restUp[2] + dz * restUp[3],
					dx * restForward[1] + dy * restForward[2] + dz * restForward[3],
				}
			end
		end
	end
end

-- Integrate angular velocity from net torque of per-node forces.
-- Called each step before constraint solve.
function World:_computeAngularVelocity()
	local nodes = self.nodes
	local count = #nodes
	local com = self.centerOfMass
	local inertia = self.inertia
	local dt = self.fixedDt
	if not (dt > 0 and dt == dt) then
		dt = 1 / 60
	end
	if self.parked then
		self.angularVelocity[1] = 0
		self.angularVelocity[2] = 0
		self.angularVelocity[3] = 0
		self.yawRate = 0
		return
	end

	-- Sum torques: τ = Σ r × F for each dynamic node
	-- n.fx/fy/fz are accumulated forces (already in Newtons, not per-mass).
	local tx, ty, tz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local fx, fy, fz = n.fx, n.fy, n.fz
			if fx == fx and fy == fy and fz == fz then -- finite
				local rx = n.x - com[1]
				local ry = n.y - com[2]
				local rz = n.z - com[3]
				-- Cross product r × F
				tx += ry * fz - rz * fy
				ty += rz * fx - rx * fz
				tz += rx * fy - ry * fx
			end
		end
	end

	-- Integrate: ω += τ/I * dt
	local av = self.angularVelocity
	av[1] = nan_guard(av[1] + tx / inertia[1] * dt, 0)
	av[2] = nan_guard(av[2] + ty / inertia[2] * dt, 0)
	av[3] = nan_guard(av[3] + tz / inertia[3] * dt, 0)

	-- Angular damping: ω *= (1 - damping * dt)
	-- Vehicle asset provides self.angular_damping; if missing, no damping (1 = no effect)
	local damping = self.angularDamping or 0
	local dampFactor = math.max(1 - damping * dt, 0)
	av[1] *= dampFactor
	av[2] *= dampFactor
	av[3] *= dampFactor

	-- Angular velocity clamp (vehicle asset provides self.angular_velocity_clamp)
	-- If missing, no clamp (infinite = no effect)
	local avClamp = self.angular_velocity_clamp
	if avClamp and avClamp > 0 then
		av[1] = clamp(av[1], -avClamp, avClamp)
		av[2] = clamp(av[2], -avClamp, avClamp)
		av[3] = clamp(av[3], -avClamp, avClamp)
	end
end

-- Apply the change in rotational velocity contribution to each dynamic node.
-- Node velocities already contain the previous omega cross r term, so adding
-- the full term every fixed step injects energy and makes pitching/rolling grow
-- without additional torque.
function World:_applyAngularMotion()
	local nodes = self.nodes
	local count = #nodes
	local com = self.centerOfMass
	local av = self.angularVelocity
	local dt = self.fixedDt
	if not (dt > 0 and dt == dt) then return end
	self.angularMotionContributions = self.angularMotionContributions or {}

	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local rx = n.x - com[1]
			local ry = n.y - com[2]
			local rz = n.z - com[3]
			-- v = ω × r
			local vx = av[2] * rz - av[3] * ry
			local vy = av[3] * rx - av[1] * rz
			local vz = av[1] * ry - av[2] * rx
			local previous = self.angularMotionContributions[i] or { 0, 0, 0 }
			n.vx += vx - previous[1]
			n.vy += vy - previous[2]
			n.vz += vz - previous[3]
			self.angularMotionContributions[i] = { vx, vy, vz }
		else
			self.angularMotionContributions[i] = { 0, 0, 0 }
		end
	end
end

-- Shape matching projection: pull dynamic nodes toward rest shape.
-- Runs after XPBD constraints to restore rigid chassis behavior.
function World:_shapeMatchingProjection()
	local nodes = self.nodes
	local count = #nodes
	if not self.restRelativePositions then return end

	-- Vehicle asset provides shape_matching_stiffness (0-1)
	-- 0 = no shape matching (fully soft), 1 = fully rigid
	-- If missing, no shape matching (0 = no effect)
	local stiffness = self.shapeMatchingStiffness or 0
	if stiffness <= 0 then return end

	-- Compute current CoM for reference frame
	local totalMass = 0
	local cx, cy, cz = 0, 0, 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local m = math.max(n.mass, 0)
			totalMass += m
			cx += n.x * m
			cy += n.y * m
			cz += n.z * m
		end
	end
	if totalMass <= 1e-8 then return end
	cx = cx / totalMass
	cy = cy / totalMass
	cz = cz / totalMass

	-- Target positions = current CoM + rotated rest relative positions
	-- For small timesteps, use current chassis orientation (no rotation integration yet)
	-- Vehicle asset may provide chassis orientation matrix via self.chassis_orientation
	local targetRight = self.chassisRight or { 1, 0, 0 }
	local targetUp = self.chassisUp or { 0, 1, 0 }
	local targetForward = self.chassisForward or { 0, 0, -1 }

	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			local rel = self.restRelativePositions[i]
			if rel then
				-- Project rest relative position onto current chassis axes
				local tx = cx + rel[1] * targetRight[1] + rel[2] * targetUp[1] + rel[3] * targetForward[1]
				local ty = cy + rel[1] * targetRight[2] + rel[2] * targetUp[2] + rel[3] * targetForward[2]
				local tz = cz + rel[1] * targetRight[3] + rel[2] * targetUp[3] + rel[3] * targetForward[3]
				-- Pull toward target
				local alpha = stiffness
				n.x = n.x + (tx - n.x) * alpha
				n.y = n.y + (ty - n.y) * alpha
				n.z = n.z + (tz - n.z) * alpha
			end
		end
	end
end

end -- Module.install

return Module