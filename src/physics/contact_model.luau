--[[
	Spring-damper contact model for terrain interaction.
	Adds force-based ground contact alongside the existing position-clamping
	collision system (collision.luau). The existing system handles static
	geometry (boxes, spheres, boundaries); this module handles terrain with
	physically-grounded spring-damper normal forces, Coulomb friction, and
	rolling resistance.
--]]

local Module = {}

local function clampNaN(v, lo, hi)
	if v == nil then return nil end
	if v ~= v then return 0 end
	if v < lo then return lo end
	if v > hi then return hi end
	return v
end

function Module.newConfig(overrides)
	assert(type(overrides) == "table", "contact parameters must be authored by the vehicle asset")
	local cs = clampNaN(overrides.contact_stiffness, 0, 1e9)
	local cd = clampNaN(overrides.contact_damping, 0, 1e8)
	local bcs = clampNaN(overrides.body_contact_stiffness, 0, 1e9)
	local bcd = clampNaN(overrides.body_contact_damping, 0, 1e8)
	local tc = clampNaN(overrides.terrain_compliance, 0, 1)
	local fc = clampNaN(overrides.friction_coefficient, 0, 5)
	local rrc = clampNaN(overrides.rolling_resistance_coeff, 0, 1)
	local tca = clampNaN(overrides.tire_contact_patch_area, 0, 1)
	for name, value in { contact_stiffness = cs, contact_damping = cd, body_contact_stiffness = bcs, body_contact_damping = bcd, terrain_compliance = tc, friction_coefficient = fc, rolling_resistance_coeff = rrc, tire_contact_patch_area = tca } do
		assert(type(value) == "number", "missing authored contact parameter " .. name)
	end
	return {
		contact_stiffness = cs, contact_damping = cd, body_contact_stiffness = bcs, body_contact_damping = bcd,
		terrain_compliance = tc, friction_coefficient = fc, rolling_resistance_coeff = rrc, tire_contact_patch_area = tca,
	}
end

function Module.newState()
	return {
		total_contact_energy_lost = 0,
	}
end

function Module.applyBodyFriction(n, normal, friction_coeff, dt)
	if dt <= 0 then return end
	local vn = n.vx * normal[1] + n.vy * normal[2] + n.vz * normal[3]
	local tvx = n.vx - vn * normal[1]
	local tvy = n.vy - vn * normal[2]
	local tvz = n.vz - vn * normal[3]
	local tangentialSpeed = math.sqrt(tvx * tvx + tvy * tvy + tvz * tvz)
	if tangentialSpeed < 1e-6 or n.mass <= 0 then return end
	local fricDecel = friction_coeff * dt / n.mass
	local factor = math.max(0, math.min(1, 1 - fricDecel))
	n.vx = vn * normal[1] + tvx * factor
	n.vy = vn * normal[2] + tvy * factor
	n.vz = vn * normal[3] + tvz * factor
end

function Module.computeRollingResistance(forward_speed, normal_force, config)
	if math.abs(forward_speed) <= 0.1 or normal_force <= 0 then return 0 end
	return config.rolling_resistance_coeff * normal_force
end

function Module.install(World)
function World:_applyContactForces(dt)
	if dt <= 0 then return end
	local nodes = self.nodes
	local count = #nodes
	if count == 0 then return end

	local cfg = self._contactConfig
	local cs = self._contactState
	if not cfg or not cs then return end

	local tireStiffness = cfg.contact_stiffness
	local tireDamping = cfg.contact_damping
	local bodyStiffness = cfg.body_contact_stiffness
	local bodyDamping = cfg.body_contact_damping
	local frictionCoeff = cfg.friction_coefficient
	local rrCoeff = cfg.rolling_resistance_coeff
	local springScale = 1 - cfg.terrain_compliance

	for i = 1, count do
		local n = nodes[i]
		if not n.fixed and n.collision then
			local terrain_h = self:terrainHeight(n.x, n.z)
			local baseH = terrain_h
			if i > 4 then
				baseH = terrain_h + 0.2
			end
			local penetration = baseH - n.y
			if penetration > 0 then
				local stiffness, damping
				if i <= 4 then
					stiffness = tireStiffness
					damping = tireDamping
				else
					stiffness = bodyStiffness
					damping = bodyDamping
					if penetration > 0.3 then
						damping = damping * (1 + (penetration - 0.3) * 10)
					end
				end

				local normal = self:terrainNormal(n.x, n.z)
				local v_normal = n.vx * normal[1] + n.vy * normal[2] + n.vz * normal[3]
				local f_spring = stiffness * penetration * springScale
				local f_damping = -damping * v_normal
				local f_normal = f_spring + f_damping
				if f_normal ~= f_normal or f_normal < 0 then
					f_normal = 0
				end

				n.fx = n.fx + f_normal * normal[1]
				n.fy = n.fy + f_normal * normal[2]
				n.fz = n.fz + f_normal * normal[3]

				local tvx = n.vx - v_normal * normal[1]
				local tvy = n.vy - v_normal * normal[2]
				local tvz = n.vz - v_normal * normal[3]
				local tangSpeed = math.sqrt(tvx * tvx + tvy * tvy + tvz * tvz)
				if tangSpeed ~= tangSpeed then
					tangSpeed = 0
				end

				if tangSpeed > 1e-6 and n.mass > 0 then
					local fricDecel = frictionCoeff * f_normal / n.mass
					local fricFactor = math.max(0, math.min(1, 1 - fricDecel * dt / tangSpeed))
					n.vx = v_normal * normal[1] + tvx * fricFactor
					n.vy = v_normal * normal[2] + tvy * fricFactor
					n.vz = v_normal * normal[3] + tvz * fricFactor
				end

				if i <= 4 and tangSpeed > 1e-6 and n.mass > 0 then
					local f_rr = rrCoeff * f_normal
					if f_rr > 0 then
						local rrDecel = f_rr / n.mass * dt
						local rrFrac = math.max(0, math.min(1, rrDecel / tangSpeed))
						n.vx = n.vx - tvx * rrFrac
						n.vy = n.vy - tvy * rrFrac
						n.vz = n.vz - tvz * rrFrac
					end
				end

				cs.total_contact_energy_lost = cs.total_contact_energy_lost + f_normal * penetration * dt
			end
		end
	end
end

function World:setupContactModel(configOverrides)
	self._contactConfig = Module.newConfig(configOverrides)
	self._contactState = Module.newState()
end

end

return Module