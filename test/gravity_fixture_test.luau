--[[
	Fixture runner for Lune (standalone Luau runtime).
	Validates the single-node-gravity fixture against the physics contract.
--]]

local fs = require("@lune/fs")
local serde = require("@lune/serde")
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

function Math.classifyDamageZone(x, y, z, centerX, centerZ)
	local dx = math.abs(x - centerX)
	local dz = z - centerZ
	if y > 1.2 then return 4
	elseif y < 0.15 then return 5
	elseif dz < -0.35 then return 1
	elseif dz > 0.35 then return 2
	elseif dx > 0.55 then return 3
	else return 0 end
end

-- ============================================================
-- Contract (inlined from src/contract/Contract.lua)
-- ============================================================
local Contract = {}
Contract.VERSION = "0.1"
Contract.DEFAULT_GRAVITY = -9.81
Contract.DEFAULT_FIXED_DT = 1 / 60
Contract.DEFAULT_MAX_SUBSTEPS = 8
Contract.TERRAIN_GRID_SIZE = 64
Contract.TERRAIN_EXTENT = 400.0
Contract.TELEMETRY_LEN = 86
Contract.T = {
	SPEED_MPS = 0, SPEED_KMH = 1, RPM = 2, GEAR = 3,
	THROTTLE = 4, BRAKE = 5, STEERING = 6, CLUTCH = 7,
	COOLANT = 8, OIL_TEMP = 9, OIL_PRESSURE = 10, TRANS_TEMP = 11,
	FUEL = 12, DERATE = 13, DAMAGE = 14, SUSPENSION_LOAD = 15,
	GRIP = 16, TERRAIN_HEIGHT = 17, WHEEL_SPEED = 18, DRIVE_TORQUE = 19,
	ENGINE_RUNNING = 20, SHIFT_PHASE = 21, ABS_ACTIVE = 22, TC_ACTIVE = 23,
	VSC_ACTIVE = 24, BROKEN_BEAMS = 25, ENGINE_STAGE = 26, TIRE_WEAR = 27,
	TIRE_TEMP = 28, CONTACTS = 29, SIM_TIME = 30, FUEL_PERCENT = 31,
	TCM_STATE = 32, TCM_LINE_PRESSURE = 33, TCM_LOCKUP = 34,
	REQUESTED_GEAR = 35, SLIP_RATIO = 36, YAW_ERROR = 37,
	LUGGING_STRESS = 38, OVERREV_STRESS = 39, RUT_DEPTH = 40,
	INPUT_SHAFT_RPM = 41, OUTPUT_SHAFT_RPM = 42, CONVERTER_COUPLING = 43,
	WHEEL_SPEED_FL = 44, WHEEL_SPEED_FR = 45, WHEEL_SPEED_RL = 46, WHEEL_SPEED_RR = 47,
	SUSP_COMPRESSION_FL = 48, SUSP_COMPRESSION_FR = 49,
	SUSP_COMPRESSION_RL = 50, SUSP_COMPRESSION_RR = 51,
	SUSP_LOAD_FL = 52, SUSP_LOAD_FR = 53, SUSP_LOAD_RL = 54, SUSP_LOAD_RR = 55,
	TIRE_TEMP_FL = 56, TIRE_TEMP_FR = 57, TIRE_TEMP_RL = 58, TIRE_TEMP_RR = 59,
	TIRE_WEAR_FL = 60, TIRE_WEAR_FR = 61, TIRE_WEAR_RL = 62, TIRE_WEAR_RR = 63,
	IMPACT_SEVERITY = 64, DEFORMATION_DEPTH = 65, BROKEN_PARTS = 66,
	DAMAGE_ZONE = 67, ENGINE_WARNING = 68,
	ADAS_FCW = 69, ADAS_AEB = 70, ADAS_CONFIDENCE = 71,
	TCM_FAULT_MASK = 72, TCM_DIAGNOSTIC_CODE = 73,
	TCM_SENSOR_INPUT_RPM = 74, TCM_SENSOR_SPEED_KMH = 75,
	TCM_SENSOR_AGE = 76, TCM_SHIFT_LATENCY = 77,
	TCM_TORQUE_REDUCTION = 78, TCM_FAIL_SAFE_GEAR = 79,
	ADAS_TARGET_DETECTED = 80, ADAS_TARGET_DISTANCE = 81,
	ADAS_TIME_TO_COLLISION = 82, ADAS_RELATIVE_SPEED = 83,
	ADAS_BRAKE_COMMAND = 84, ADAS_SENSOR_FAULTS = 85,
}

-- ============================================================
-- World (inlined, minimal for fixture validation)
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
	self.gravity = Contract.DEFAULT_GRAVITY
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

function World:setNodeCollision(nodeId, collision)
	local node = self.nodes[nodeId + 1]
	if node then
		node.collision = (nodeId >= 4) and (not node.fixed) and collision
	end
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

function World:addTriangle(a, b, c)
	local na = self.nodes[a + 1]
	local nb = self.nodes[b + 1]
	local nc = self.nodes[c + 1]
	if not (na and nb and nc) then return end
	local abx, aby, abz = nb.x - na.x, nb.y - na.y, nb.z - na.z
	local acx, acy, acz = nc.x - na.x, nc.y - na.y, nc.z - na.z
	local crx = aby * acz - abz * acy
	local cry = abz * acx - abx * acz
	local crz = abx * acy - aby * acx
	local area = 0.5 * math.sqrt(crx * crx + cry * cry + crz * crz)
	if area > 1e-8 then
		table.insert(self.triangles, { a = a, b = b, c = c, restArea = area, lambda = 0 })
	end
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

function World:_updateTelemetry()
	local t = self.telemetry
	local avgVx, avgVz = 0, 0
	local dynCount = 0
	for i = 1, #self.nodes do
		local n = self.nodes[i]
		if not n.fixed then avgVx += n.vx; avgVz += n.vz; dynCount += 1 end
	end
	if dynCount > 0 then avgVx /= dynCount; avgVz /= dynCount end
	local speed = math.sqrt(avgVx * avgVx + avgVz * avgVz)
	t[Contract.T.SPEED_MPS + 1] = speed
	t[Contract.T.SPEED_KMH + 1] = speed * 3.6
	t[Contract.T.SIM_TIME + 1] = self.time
end

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
-- Fixture runner
-- ============================================================
local function resolvePath(obj, path)
	local current = obj
	for part in string.gmatch(path, "[^%.]+") do
		local name, index = string.match(part, "(.+)%[(%d+)%]")
		if name then
			current = current[name]
			if current then current = current[tonumber(index) + 1] end
		else
			current = current[part]
		end
		if current == nil then return nil end
	end
	return current
end

local function runAssertion(assertion, data)
	local actual = resolvePath(data, assertion.path)
	if actual == nil then
		return false, string.format("  FAIL: %s = nil", assertion.path)
	end
	local op = assertion.operator
	local expected = assertion.value
	if op == "approximatelyEquals" then
		local tol = assertion.absoluteTolerance or 0.001
		if math.abs(actual - expected) > tol then
			return false, string.format("  FAIL: %s = %.10f (expected ~%.10f +/- %.10f)", assertion.path, actual, expected, tol)
		end
	elseif op == "lessThan" then
		if actual >= expected then
			return false, string.format("  FAIL: %s = %.10f (expected < %.10f)", assertion.path, actual, expected)
		end
	elseif op == "greaterThan" then
		if actual <= expected then
			return false, string.format("  FAIL: %s = %.10f (expected > %.10f)", assertion.path, actual, expected)
		end
	elseif op == "equals" then
		if actual ~= expected then
			return false, string.format("  FAIL: %s = %s (expected %s)", assertion.path, tostring(actual), tostring(expected))
		end
	else
		return false, string.format("  SKIP: unknown op %s", op)
	end
	return true, string.format("  PASS: %s = %.10f", assertion.path, actual)
end

-- ============================================================
-- Main
-- ============================================================
local fixturePath = "fixtures/engine/single-node-gravity.json"
if #process.args > 1 then
	fixturePath = process.args[2]
end

print(string.format("Fixture: %s", fixturePath))
local fixture = serde.decode("json", fs.readFile(fixturePath))

local world = World.new()
world.gravity = fixture.initial.gravityMps2
world.fixedDt = fixture.initial.fixedDtSeconds

for _, nd in fixture.initial.nodes do
	world:addNode(nd.id, nd.positionM[1], nd.positionM[2], nd.positionM[3], nd.massKg, nd.fixed)
	if nd.collision ~= nil then world:setNodeCollision(nd.id, nd.collision) end
end
for _, bd in (fixture.initial.beams or {}) do
	world:addBeam(bd.id, bd.nodeA, bd.nodeB, bd.stiffness, bd.damping, bd.strength)
end
for _, td in (fixture.initial.triangles or {}) do
	world:addTriangle(td.nodeA, td.nodeB, td.nodeC)
end

print(string.format("Nodes: %d  Beams: %d  Triangles: %d", #world.nodes, #world.beams, #world.triangles))

for i, sd in fixture.steps do
	world:step(sd.elapsedSeconds)
end

for _, n in world.nodes do
	print(string.format("  node[%d] pos=(%.10f, %.10f, %.10f) vel=(%.10f, %.10f, %.10f)",
		n.id, n.x, n.y, n.z, n.vx, n.vy, n.vz))
end

-- Build snapshot matching fixture format for assertions
local snapshot = { nodes = {} }
for _, n in world.nodes do
	snapshot.nodes[n.id + 1] = {
		positionM = { n.x, n.y, n.z },
		velocityMps = { n.vx, n.vy, n.vz },
	}
end

print("\n--- Assertions ---")
local allPassed = true
for _, a in fixture.assertions do
	local ok, msg = runAssertion(a, snapshot)
	print(msg)
	if not ok then allPassed = false end
end

if allPassed then
	print("\nOK ALL PASSED")
else
	print("\nFAILED")
	process.exit(1)
end
