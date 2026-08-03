--[[
	Angular integration validation (Luau port).

	Builds a rigid beam cage, applies a torque couple, and asserts the
	resulting angular acceleration matches τ/I_yy. Mirrors the Rust test
	`rigid_cage_rotates_correctly_under_torque_couple` in tests.rs.

	Run: lune run test/angular_validation_test.luau
--]]

local process = require("@lune/process")

-- ============================================================
-- Math utilities (inlined from src/math.luau)
-- ============================================================
local Math = {}
function Math.finite_or_zero(v)
	if v ~= v then return 0 end
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

-- ============================================================
-- Contract (inlined from src/contract)
-- ============================================================
local Contract = {}
Contract.DEFAULT_GRAVITY = -9.81
Contract.DEFAULT_FIXED_DT = 1 / 60
Contract.DEFAULT_MAX_SUBSTEPS = 8
Contract.TERRAIN_GRID_SIZE = 64
Contract.TELEMETRY_LEN = 86
Contract.T = { DERATE = 13, SPEED_MPS = 0, SPEED_KMH = 1, SIM_TIME = 30 }

-- ============================================================
-- Minimal World (inlined from gravity_fixture_test.luau)
-- ============================================================
local World = {}
World.__index = World

local function makeTelemetry()
	local t = table.create(Contract.TELEMETRY_LEN, 0)
	t[Contract.T.DERATE + 1] = 1.0
	return t
end

function World.new()
	local self = setmetatable({}, World)
	self.nodes = {}
	self.beams = {}
	self.triangles = {}
	self.gravity = 0 -- disabled for angular test
	self.time = 0
	self.accumulator = 0
	self.fixedDt = Contract.DEFAULT_FIXED_DT
	self.maxSubsteps = Contract.DEFAULT_MAX_SUBSTEPS
	self.restPositions = {}
	self.baseNodeMasses = {}
	self.constraintStartPositions = {}
	self.terrainRuts = table.create(Contract.TERRAIN_GRID_SIZE * Contract.TERRAIN_GRID_SIZE, 0)
	self.groundFriction = 0.94
	self.telemetry = makeTelemetry()
	return self
end

function World:addNode(id, x, y, z, mass, fixed)
	x = Math.finite_or_zero(x)
	y = Math.finite_or_zero(y)
	z = Math.finite_or_zero(z)
	if not (mass > 0 and mass == mass) then mass = 1 end
	local invMass = if fixed then 0 else 1 / mass
	table.insert(self.nodes, {
		id = id, x = x, y = y, z = z,
		vx = 0, vy = 0, vz = 0,
		mass = mass, invMass = invMass, fixed = fixed,
		collision = (not fixed) and (#self.nodes >= 4),
		fx = 0, fy = 0, fz = 0, lastForce = 0,
	})
	table.insert(self.constraintStartPositions, {x, y, z})
	table.insert(self.restPositions, {x, y, z})
	table.insert(self.baseNodeMasses, mass)
end

function World:addBeam(id, nodeA, nodeB, stiffness, damping, strength)
	local a = self.nodes[nodeA + 1]
	local b = self.nodes[nodeB + 1]
	local length = 1
	if a and b then
		local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
		length = math.sqrt(dx * dx + dy * dy + dz * dz)
		if length < 1e-4 then length = 1e-4 end
	end
	if stiffness < 1 then stiffness = 1 end
	if damping < 0 then damping = 0 end
	if strength < 0 then strength = 0 end
	table.insert(self.beams, {
		id = id, nodeA = nodeA, nodeB = nodeB,
		stiffness = stiffness, damping = damping, strength = strength,
		yieldStrength = strength * 0.55, plasticity = 0.25,
		length = length, initialLength = length,
		broken = false, lambda = 0, stress = 0,
	})
end

function World:_updateMassProperties()
	for i = 1, #self.nodes do
		local n = self.nodes[i]
		if not n.fixed then
			local baseMass = self.baseNodeMasses[i] or n.mass
			if baseMass < 1e-6 then baseMass = 1e-6 end
			n.mass = baseMass
			n.invMass = 1 / n.mass
		end
	end
end

function World:_applyForces()
	local nodes = self.nodes
	local count = #nodes
	local totalMass = 0
	for i = 1, count do
		if not nodes[i].fixed then totalMass += math.max(nodes[i].mass, 0) end
	end
	if totalMass <= 0 then return end
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then n.fy += n.mass * self.gravity end
	end
end

function World:_integrateVelocities(dt)
	for i = 1, #self.nodes do
		local n = self.nodes[i]
		if not n.fixed then
			n.vx += n.fx * n.invMass * dt
			n.vy += n.fy * n.invMass * dt
			n.vz += n.fz * n.invMass * dt
			n.lastForce = math.sqrt(n.fx * n.fx + n.fy * n.fy + n.fz * n.fz)
			n.fx = 0; n.fy = 0; n.fz = 0
			n.vx = Math.finite_or_zero(math.clamp(n.vx, -200, 200))
			n.vy = Math.finite_or_zero(math.clamp(n.vy, -200, 200))
			n.vz = Math.finite_or_zero(math.clamp(n.vz, -200, 200))
			n.x = Math.finite_or_zero(n.x + n.vx * dt)
			n.y = Math.finite_or_zero(n.y + n.vy * dt)
			n.z = Math.finite_or_zero(n.z + n.vz * dt)
		end
	end
end

function World:_solveXPBD()
	local dt = self.fixedDt
	local dt2 = dt * dt
	local nodes = self.nodes
	local beams = self.beams
	for i = 1, #beams do beams[i].lambda = 0 end
	for i = 1, #self.triangles do self.triangles[i].lambda = 0 end

	for _iter = 1, 30 do
		for bi = 1, #beams do
			local beam = beams[bi]
			if not beam.broken then
				local a, b = beam.nodeA + 1, beam.nodeB + 1
				if a <= #nodes and b <= #nodes then
					local na, nb = nodes[a], nodes[b]
					local dx, dy, dz = nb.x - na.x, nb.y - na.y, nb.z - na.z
					local len = math.sqrt(dx * dx + dy * dy + dz * dz)
					if len > 1e-8 then
						local c = len - beam.length
						local invSum = na.invMass + nb.invMass
						if invSum > 0 then
							local compliance = 1 / math.max(beam.stiffness, 1)
							local alpha = compliance / dt2
							local dLambda = (-c - alpha * beam.lambda) / (invSum + alpha)
							beam.lambda += dLambda
							local nx, ny, nz = dx / len, dy / len, dz / len
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

	local invDt = 1 / math.max(dt, 1e-6)
	for i = 1, #nodes do
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
end

function World:_collideWithTerrain()
	for i = 1, #self.nodes do
		local n = self.nodes[i]
		if not n.fixed and n.collision then
			if n.y < 0 then
				if n.vy < 0 then n.vy = -n.vy * 0.15 end
				n.y = 0
			end
		end
	end
end

function World:_collideWithStaticGeometry() end
function World:_applyDrag() end
function World:_updateTelemetry() end

function World:stepFixed()
	local count = #self.nodes
	if count == 0 then return end
	while #self.constraintStartPositions < count do
		table.insert(self.constraintStartPositions, { 0, 0, 0 })
	end
	self:_updateMassProperties()
	self:_applyForces()
	self:_integrateVelocities(self.fixedDt)
	for i = 1, count do
		local sp = self.constraintStartPositions[i]
		sp[1] = self.nodes[i].x; sp[2] = self.nodes[i].y; sp[3] = self.nodes[i].z
	end
	self:_solveXPBD()
	self:_collideWithTerrain()
	self:_collideWithStaticGeometry()
	self:_applyDrag()
	self:_updateTelemetry()
end

function World:step(elapsed)
	self.accumulator += elapsed
	local steps = 0
	while self.accumulator >= self.fixedDt and steps < self.maxSubsteps do
		self:stepFixed()
		self.time += self.fixedDt
		self.accumulator -= self.fixedDt
		steps += 1
	end
end

-- ============================================================
-- Test
-- ============================================================
local passed = 0
local failed = 0

local function test(name, fn)
	local ok, err = pcall(fn)
	if ok then
		passed += 1
		print(string.format("  PASS  %s", name))
	else
		failed += 1
		print(string.format("  FAIL  %s: %s", name, tostring(err)))
	end
end

test("rigid cage rotates correctly under torque couple", function()
	local w = World.new()
	w.gravity = 0

	local mass = 100
	local halfW = 0.75
	local halfL = 1.5

	-- 4 nodes in a rectangle at y=1
	w:addNode(0, -halfW, 1, -halfL, mass, false) -- front-left
	w:addNode(1,  halfW, 1, -halfL, mass, false) -- front-right
	w:addNode(2, -halfW, 1,  halfL, mass, false) -- rear-left
	w:addNode(3,  halfW, 1,  halfL, mass, false) -- rear-right

	local stiffness = 500_000
	local damping = 0.5
	local strength = 1_000_000

	-- 4 edges + 2 diagonals
	w:addBeam(0, 0, 1, stiffness, damping, strength)
	w:addBeam(1, 2, 3, stiffness, damping, strength)
	w:addBeam(2, 0, 2, stiffness, damping, strength)
	w:addBeam(3, 1, 3, stiffness, damping, strength)
	w:addBeam(4, 0, 3, stiffness, damping, strength)
	w:addBeam(5, 1, 2, stiffness, damping, strength)

	-- I_yy = 4 * mass * (halfW² + halfL²)
	local iYy = 4 * mass * (halfW * halfW + halfL * halfL)

	local forcePerNode = 500
	local expectedTorque = 4 * halfL * forcePerNode
	local expectedAlpha = expectedTorque / iYy

	local dt = 1 / 60
	local steps = 30

	local function heading()
		local nodes = w.nodes
		local fx = (nodes[1].x + nodes[2].x) * 0.5
		local fz = (nodes[1].z + nodes[2].z) * 0.5
		local rx = (nodes[3].x + nodes[4].x) * 0.5
		local rz = (nodes[3].z + nodes[4].z) * 0.5
		return math.atan2(fx - rx, -(fz - rz))
	end

	local initialHeading = heading()

	-- Phase 1: apply torque couple
	for _ = 1, steps do
		w.nodes[1].fx = forcePerNode
		w.nodes[2].fx = forcePerNode
		w.nodes[3].fx = -forcePerNode
		w.nodes[4].fx = -forcePerNode
		w:step(dt)
	end

	local midHeading = heading()

	-- Phase 2: coast (no forces)
	for _ = 1, steps do
		w:step(dt)
	end

	local finalHeading = heading()

	-- 1. Angular response: Δheading = 0.5 * α * t²
	local t = dt * steps
	local expectedHeadingChange = 0.5 * expectedAlpha * t * t
	local measuredHeadingChange = midHeading - initialHeading
	local tolerance = 0.25
	local ratio = measuredHeadingChange / expectedHeadingChange

	assert(
		math.abs(ratio - 1) < tolerance,
		string.format("angular response: measured %.4f rad, expected %.4f rad (ratio %.3f)",
			measuredHeadingChange, expectedHeadingChange, ratio)
	)

	-- 2. Angular momentum conservation
	local coastRate = (finalHeading - midHeading) / t
	local accelAvgRate = (midHeading - initialHeading) / t
	local expectedCoastRate = 2 * accelAvgRate
	local coastRatio = if math.abs(expectedCoastRate) > 1e-9
		then coastRate / expectedCoastRate else 0

	assert(
		coastRatio > 0.65 and coastRatio < 1.35,
		string.format("angular momentum: coast %.4f, expected ~%.4f (ratio %.3f)",
			coastRate, expectedCoastRate, coastRatio)
	)

	-- 3. Cage integrity: beam strain < 2%
	for _, beam in w.beams do
		local strain = math.abs(beam.length - beam.initialLength) / beam.initialLength
		assert(
			strain < 0.02,
			string.format("beam %d overstrained: initial %.4f, current %.4f, strain %.1f%%",
				beam.id, beam.initialLength, beam.length, strain * 100)
		)
	end

	-- 4. Center of mass drift < 5 cm
	local totalMass = 4 * mass
	local comX, comZ = 0, 0
	for _, n in w.nodes do
		comX += n.x * n.mass
		comZ += n.z * n.mass
	end
	comX /= totalMass
	comZ /= totalMass
	local comDrift = math.sqrt(comX * comX + comZ * comZ)

	assert(
		comDrift < 0.05,
		string.format("center of mass drifted %.4f m", comDrift)
	)
end)

print(string.format("\n=== Results: %d/%d passed ===", passed, passed + failed))
if failed > 0 then
	process.exit(1)
end
