--[[
	Solver hardening for XPBD constraint solver.
	Overrides World:_solveXPBD with configurable iterations, warm starting,
	velocity damping, anti-tunnel clamping, and adaptive compliance.

	Install after constraints.luau so the override replaces the original.
--]]

local Module = {}

local function clamp(value, lo, hi)
	if value < lo then return lo end
	if value > hi then return hi end
	return value
end

-- ponytail: Adaptive compliance thresholds are tuned for a 0.0167s fixed dt.
-- If fixedDt changes significantly, rescale stressLo/stressHi.

function Module.install(World, Math)

-- Default solver tuning (set on World if not already present)
if World.solverIterations == nil then World.solverIterations = 40 end
if World.maxSolverIterations == nil then World.maxSolverIterations = 80 end
if World.velocityDamping == nil then World.velocityDamping = 0.99995 end
if World.previousBeamLambdas == nil then World.previousBeamLambdas = {} end
if World.previousTriLambdas == nil then World.previousTriLambdas = {} end

-- Adaptive compliance thresholds (lambda/dt² stress measure)
local ADAPTIVE_STRESS_LO = 50      -- below this: softer compliance for better convergence
local ADAPTIVE_STRESS_HI = 5000    -- above this: stiffer compliance to resist load
local COMPLIANCE_FLOOR = 1e-7      -- minimum compliance (stiffest)
local COMPLIANCE_CEIL = 1e-3       -- maximum compliance (softest)
local BASE_BEAM_COMPLIANCE = 1     -- = 1/max(stiffness, 1) baseline

local function computeAdaptiveCompliance(stress, dt2)
	-- Map stress to compliance: high stress → stiff, low stress → soft
	local t
	if stress < ADAPTIVE_STRESS_LO then
		t = 0
	elseif stress > ADAPTIVE_STRESS_HI then
		t = 1
	else
		t = (stress - ADAPTIVE_STRESS_LO) / (ADAPTIVE_STRESS_HI - ADAPTIVE_STRESS_LO)
	end
	-- Interpolate between soft and stiff
	return COMPLIANCE_CEIL + (COMPLIANCE_FLOOR - COMPLIANCE_CEIL) * t
end

function World:_solveXPBD()
	local nodes = self.nodes
	local beams = self.beams
	local triangles = self.triangles
	local nodeCount = #nodes
	local beamCount = #beams
	local triCount = #triangles

	local dt = self.fixedDt
	if not (dt > 0 and dt == dt) then dt = 1 / 60 end
	local dt2 = dt * dt
	local invDt = 1 / math.max(dt, 1e-6)

	-- Clamp iterations to configured bounds
	local iterations = self.solverIterations or 40
	iterations = math.min(iterations, self.maxSolverIterations or 80)
	iterations = math.max(iterations, 1)

	-- Warm starting: seed lambdas from previous frame for faster convergence
	local prevBeam = self.previousBeamLambdas
	local prevTri = self.previousTriLambdas
	for i = 1, beamCount do
		local beam = beams[i]
		if i <= #prevBeam and not beam.broken then
			beam.lambda = prevBeam[i] * 0.8  -- damped warm start (80% carryover)
		else
			beam.lambda = 0
		end
	end
	for i = 1, triCount do
		if i <= #prevTri then
			triangles[i].lambda = prevTri[i] * 0.8
		else
			triangles[i].lambda = 0
		end
	end
	self.volumeLambda = 0

	local function clamp(v, lo, hi)
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

local function solveBeamProjection()
		for bi = 1, beamCount do
			local beam = beams[bi]
			if not beam.broken then
				local a = beam.nodeA + 1
				local b = beam.nodeB + 1
				if a <= nodeCount and b <= nodeCount then
					local na = nodes[a]
					local nb = nodes[b]
					local dx = nb.x - na.x
					local dy = nb.y - na.y
					local dz = nb.z - na.z
					local len = math.sqrt(dx*dx + dy*dy + dz*dz)
					if len > 1e-8 then
						local c = len - beam.length
						local invSum = na.invMass + nb.invMass
						if invSum > 0 then
							-- Adaptive compliance: stiffer under load, softer near rest
							local stress = math.abs(beam.lambda / math.max(dt2, 1e-12))
							local compliance = computeAdaptiveCompliance(stress, dt2)
							local alpha = compliance / dt2
							local dLambda = (-c - alpha * beam.lambda) / (invSum + alpha)
							-- NaN guard
							if dLambda ~= dLambda then dLambda = 0 end
							beam.lambda += dLambda
							local nx, ny, nz = dx/len, dy/len, dz/len
							if not na.fixed then
								na.x -= dLambda * na.invMass * nx
								na.y -= dLambda * na.invMass * ny
								na.z -= dLambda * na.invMass * nz
							end
							if not nb.fixed then
								nb.x += dLambda * nb.invMass * nx
								nb.y += dLambda * nb.invMass * ny
								nb.z += dLambda * nb.invMass * nz
							end
						end
					end
				end
			end
		end
	end

	local function solveTriangleArea()
		for ti = 1, triCount do
			local tri = triangles[ti]
			local ai, bi2, ci = tri.a + 1, tri.b + 1, tri.c + 1
			if ai <= nodeCount and bi2 <= nodeCount and ci <= nodeCount then
				local a = nodes[ai]
				local b = nodes[bi2]
				local c = nodes[ci]
				local abx, aby, abz = b.x - a.x, b.y - a.y, b.z - a.z
				local acx, acy, acz = c.x - a.x, c.y - a.y, c.z - a.z
				local crx, cry, crz = Math.cross3({abx,aby,abz},{acx,acy,acz})
				local doubleArea = math.sqrt(crx*crx + cry*cry + crz*crz)
				if doubleArea >= 1e-10 then
					local nn = { crx/doubleArea, cry/doubleArea, crz/doubleArea }
					local gradAx, gradAy, gradAz = Math.scale3(Math.cross3({b.x-c.x,b.y-c.y,b.z-c.z}, nn), 0.5)
					local gradBx, gradBy, gradBz = Math.scale3(Math.cross3({c.x-a.x,c.y-a.y,c.z-a.z}, nn), 0.5)
					local gradCx, gradCy, gradCz = Math.scale3(Math.cross3(nn, {abx,aby,abz}), 0.5)
					local area = doubleArea * 0.5
					local error_ = area - tri.restArea
					local compliance = 1e-5 / math.max(dt2, 1e-12)
					local wSum = a.invMass * (gradAx*gradAx+gradAy*gradAy+gradAz*gradAz)
						+ b.invMass * (gradBx*gradBx+gradBy*gradBy+gradBz*gradBz)
						+ c.invMass * (gradCx*gradCx+gradCy*gradCy+gradCz*gradCz)
					local denom = wSum + compliance
					if denom > 1e-12 then
						local dL = (-error_ - compliance * tri.lambda) / denom
						if dL ~= dL then dL = 0 end
						tri.lambda += dL
						if a.invMass > 0 then
							a.x += dL * a.invMass * gradAx
							a.y += dL * a.invMass * gradAy
							a.z += dL * a.invMass * gradAz
						end
						if b.invMass > 0 then
							b.x += dL * b.invMass * gradBx
							b.y += dL * b.invMass * gradBy
							b.z += dL * b.invMass * gradBz
						end
						if c.invMass > 0 then
							c.x += dL * c.invMass * gradCx
							c.y += dL * c.invMass * gradCy
							c.z += dL * c.invMass * gradCz
						end
					end
				end
			end
		end
	end

	-- Solver iteration loop
	for _iter = 1, iterations do
		solveBeamProjection()
		solveTriangleArea()
		self:_solveBodyAttachmentConstraints()
		self:_solveVolumePreservation()
	end

	-- Beam damping (once after all iterations)
	self:_applyBeamDamping(dt)

	-- Feed constraint correction into velocity
	for i = 1, nodeCount do
		local n = nodes[i]
		if not n.fixed then
			local sp = self.constraintStartPositions[i]
			if sp then
				n.vx += (n.x - sp[1]) * invDt
				n.vy += (n.y - sp[2]) * invDt
				n.vz += (n.z - sp[3]) * invDt
			end
		end
	end

	-- Velocity damping per substep
	self:_applyVelocityDamping()

	-- Anti-tunnel: clamp nodes below terrain
	self:_antiTunnelClamp()

	-- Save lambdas for next frame's warm starting
	-- (store after all corrections so the carryover is meaningful)
	for i = 1, beamCount do
		prevBeam[i] = beams[i].lambda
	end
	for i = 1, triCount do
		prevTri[i] = triangles[i].lambda
	end
end

-- Velocity damping: exponential decay per substep to absorb micro-vibrations.
function World:_applyVelocityDamping()
	local d = self.velocityDamping or 0.99995
	-- NaN guard
	if d ~= d then d = 0.99995 end
	local nodes = self.nodes
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			n.vx = n.vx * d
			n.vy = n.vy * d
			n.vz = n.vz * d
		end
	end
end

-- Anti-tunnel: prevent non-fixed, non-wheel nodes from sinking below terrain.
function World:_antiTunnelClamp()
	local nodes = self.nodes
	-- Wheel nodes are indices 1-4 (suspension handles their ground contact)
	local WHEEL_COUNT = 4
	for i = WHEEL_COUNT + 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			local th = self:terrainHeight(n.x, n.z)
			if th ~= th then th = 0 end -- NaN guard on terrain query
			if n.y < th then
				local penetration = th - n.y
				n.y = th + 0.01
				-- Reflect vertical velocity with energy loss
				if n.vy < 0 then
					n.vy = -n.vy * 0.3
				end
				-- Horizontal friction scales with penetration depth: deeper
				-- penetration → more friction (harder to slide through terrain).
				local frictionFactor = clamp(1 - penetration * 2, 0.5, 1.0)
				n.vx = n.vx * frictionFactor
				n.vz = n.vz * frictionFactor
			end
		end
	end
end

end -- Module.install

return Module
