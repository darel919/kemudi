local Module = {}

function Module.install(World, Suspension, Tires, Drivetrain, Safety, Math)
-- Vehicle system update — called each fixed step before external forces.
-- The wheel nodes are suspension mounts. Tire contact, powertrain output,
-- braking, and anti-roll transfer all enter the same node force accumulator so
-- weight transfer and chassis deformation emerge from the beam graph.
function World:_updateVehicleSystems()
	if not self.drivetrain or not self.suspension_config or #self.nodes < 4 then return end
	local dt = self.fixedDt
	local nodes = self.nodes
	local d = self.drivetrain
	local controls = self.controls or {}
	local engineOn = controls.engine_on == true

	self:_updateMassProperties()

	local totalMass, avgVx, avgVy, avgVz = 0, 0, 0, 0
	for i = 1, #nodes do
		local n = nodes[i]
		if not n.fixed then
			totalMass += n.mass
			avgVx += n.vx * n.mass
			avgVy += n.vy * n.mass
			avgVz += n.vz * n.mass
		end
	end
	totalMass = math.max(totalMass, 1)
	avgVx, avgVy, avgVz = avgVx / totalMass, avgVy / totalMass, avgVz / totalMass
	local speed = math.sqrt(avgVx * avgVx + avgVz * avgVz)

	local frontX = (nodes[1].x + nodes[2].x) * 0.5
	local frontZ = (nodes[1].z + nodes[2].z) * 0.5
	local rearX = (nodes[3].x + nodes[4].x) * 0.5
	local rearZ = (nodes[3].z + nodes[4].z) * 0.5
	local frameX, frameZ = frontX - rearX, frontZ - rearZ
	local frameLength = math.sqrt(frameX * frameX + frameZ * frameZ)
	if frameLength < 1e-6 then frameX, frameZ = 0, -1 else frameX, frameZ = frameX / frameLength, frameZ / frameLength end
	local rightX, rightZ = -frameZ, frameX
	local heading = math.atan2(frameX, -frameZ)
	self.yawRate = (heading - self.previousHeading) / math.max(dt, 1e-6)
	self.previousHeading = heading

	local inputSteering = math.clamp(controls.steering or 0, -1, 1) * self.steering_config.max_steering_angle
	self.steering_angle = Suspension.updateSteering(
		self.steering_config, inputSteering, speed, self.steering_angle, dt)
	local leftSteer, rightSteer = Suspension.ackermannAngles(
		self.steering_angle, self.steering_config.wheelbase, self.steering_config.track_width)
	self.steering_states[1], self.steering_states[2] = leftSteer, rightSteer

	local function isDriven(index)
		if self.driveLayout == 1 then return index <= 2 end
		if self.driveLayout == 2 then return true end
		return index >= 3
	end

	local wheelRadius = math.max(d.wheel_radius, 0.05)
	local drivenVelocity, drivenCount = 0, 0
	for i = 1, 4 do
		if isDriven(i) then
			local n = nodes[i]
			drivenVelocity += n.vx * frameX + n.vz * frameZ
			drivenCount += 1
		end
	end
	if engineOn and drivenCount > 0 then d.wheel_speed = drivenVelocity / drivenCount / wheelRadius end

	local wheelAngularSpeeds = { 0, 0, 0, 0 }
	local wheelForwardSpeeds = { 0, 0, 0, 0 }
	local wheelAxes = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }
	local wheelRightAxes = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }
	for i = 1, 4 do
		local steer = if i == 1 then leftSteer elseif i == 2 then rightSteer else 0
		local c, s = math.cos(steer), math.sin(steer)
		local wheelX = frameX * c + rightX * s
		local wheelZ = frameZ * c + rightZ * s
		wheelAxes[i] = { wheelX, 0, wheelZ }
		wheelRightAxes[i] = { -wheelZ, 0, wheelX }
		local n = nodes[i]
		local groundSpeed = n.vx * wheelX + n.vz * wheelZ
		wheelForwardSpeeds[i] = groundSpeed
		wheelAngularSpeeds[i] = if isDriven(i) and engineOn then d.wheel_speed else groundSpeed / math.max(self.suspension_config.wheels[i].tire_radius, 0.05)
	end

	local wheelSpeedsForSafety = { 0, 0, 0, 0 }
	for i = 1, 4 do wheelSpeedsForSafety[i] = math.abs(wheelAngularSpeeds[i] * self.suspension_config.wheels[i].tire_radius) end
	local drivenIndices = if self.driveLayout == 1 then { 0, 1 } elseif self.driveLayout == 2 then { 0, 1, 2, 3 } else { 2, 3 }
	local tcModifier = 1.0
	local brakePressures = { controls.brake or 0, controls.brake or 0, controls.brake or 0, controls.brake or 0 }
	local vscTorqueModifier = 1.0
	if self.safety then
		tcModifier = Safety.updateTractionControl(
			self.safety.traction_control, wheelSpeedsForSafety, speed, drivenIndices, controls.throttle or 0, dt)
		brakePressures = Safety.updateABS(
			self.safety.abs, wheelSpeedsForSafety, speed, controls.brake or 0, dt)
		local desiredYaw = speed * math.tan(self.steering_angle) / math.max(self.steering_config.wheelbase, 0.1)
		local _, torqueModifier = Safety.updateVSC(
			self.safety.vsc, self.yawRate, desiredYaw, self.steering_angle, speed, dt)
		vscTorqueModifier = torqueModifier
	end

	d.throttle_input = if engineOn then math.clamp((controls.throttle or 0) * tcModifier, 0, 1) else 0
	d.brake_input = 0 -- service brakes are distributed per contact patch below
	d.clutch_input = math.clamp(controls.clutch or 0, 0, 1)
	if controls.gear_up then Drivetrain.requestShiftUp(d) end
	if controls.gear_down then Drivetrain.requestShiftDown(d) end
	if engineOn then
		Drivetrain.update(dt, self.vehicle_mass, d)
	else
		d.last_drive_torque = 0
		d.engine.rpm = 0
		d.is_stalled = false
	end

	-- Automatic transmissions are scheduled by the TCM after the drivetrain
	-- observes the current engine/output state. This keeps launch protection,
	-- kickdown, lockup, limp mode, and shift latency in one controller instead
	-- of running the old RPM-only threshold helper in parallel.
	if d.transmission.mode == "Automatic" and d.tcm and d.tcm.enabled then
		local speedKmh = speed * 3.6
		local target = Drivetrain.updateTCM(
			d.tcm,
			d.engine.rpm,
			speedKmh,
			controls.throttle or 0,
			controls.brake or 0,
			0,
			d.transmission_temperature or 20,
			d.transmission.current_gear,
			#d.transmission.gear_ratios,
			dt)
		d.shift_duration = 0.15 * d.tcm.shift_latency
		if d.tcm.limp_mode then
			target = Drivetrain.tcmFailSafeGear(d.tcm, #d.transmission.gear_ratios)
		end
		if target and target > d.transmission.current_gear then
			Drivetrain.requestShiftUp(d)
		elseif target and target < d.transmission.current_gear then
			local ratio = if target >= 1 then d.transmission.gear_ratios[target] * d.transmission.final_drive else d.transmission.reverse_ratio * d.transmission.final_drive
			local projectedRpm = math.abs(d.wheel_speed * ratio) * 60 / (2 * math.pi)
			if projectedRpm < d.max_downshift_rpm then Drivetrain.requestShiftDown(d) end
		end
		if d.tcm.converter_lockup then
			d.converter_coupling = 1.0
		else
			d.converter_coupling = math.clamp(speedKmh / math.max(d.tcm.lockup_speed_threshold, 5), 0.15, 0.95)
		end
	end

	local wheelLoads = { 0, 0, 0, 0 }
	local wheelGrip = { 0, 0, 0, 0 }
	local wheelLateral = { 0, 0, 0, 0 }
	local wheelLongCapacity = { 0, 0, 0, 0 }
	local wheelSurfaceRoughness = { 0, 0, 0, 0 }
	local wheelNormals = { { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 } }
	local grounded = { false, false, false, false }
	for i = 1, 4 do
		local n = nodes[i]
		local cfg = self.suspension_config.wheels[i] or Suspension.newWheelConfig()
		local surfaceFriction, surfaceRoughness, surfaceMoisture, surfaceCompactness = self:surfacePropertiesAt(n.x, n.z)
		local terrainGrip = math.clamp(
			surfaceFriction * (1 - surfaceMoisture * 0.55) * (0.75 + surfaceCompactness * 0.25), 0, 2)
		wheelSurfaceRoughness[i] = surfaceRoughness
		local terrainHeight = self:terrainHeight(n.x, n.z)
		local suspensionLength = n.y - terrainHeight - cfg.tire_radius
		local state = self.suspension_states[i]
		local previous = state.previous_suspension_length or suspensionLength
		local lengthVelocity = Suspension.suspensionLengthVelocity(suspensionLength, previous, dt)
		local maxRate = math.max(cfg.travel / math.max(dt, 1e-6), 0.5)
		lengthVelocity = math.clamp(lengthVelocity, -maxRate, maxRate)
		local result, compression, airborne = Suspension.raycastWheelWithNormal(
			cfg, { n.x, n.y, n.z }, lengthVelocity, terrainHeight, self:terrainNormal(n.x, n.z))
		local load = result.force
		if not airborne then
			load += Suspension.progressiveBumpStopForce(compression, cfg.travel, self.suspension_config.bump_stop_rate)
			grounded[i] = true
			local normal = result.normal
			wheelNormals[i] = normal
			local axis = wheelAxes[i]
			local tangentX = axis[1] - normal[1] * (axis[1] * normal[1] + axis[2] * normal[2] + axis[3] * normal[3])
			local tangentY = axis[2] - normal[2] * (axis[1] * normal[1] + axis[2] * normal[2] + axis[3] * normal[3])
			local tangentZ = axis[3] - normal[3] * (axis[1] * normal[1] + axis[2] * normal[2] + axis[3] * normal[3])
			local tangentLength = math.sqrt(tangentX * tangentX + tangentY * tangentY + tangentZ * tangentZ)
			if tangentLength > 1e-6 then tangentX, tangentY, tangentZ = tangentX / tangentLength, tangentY / tangentLength, tangentZ / tangentLength end
			wheelAxes[i] = { tangentX, tangentY, tangentZ }
			-- Build the lateral tangent from forward × contact normal. The old
			-- horizontal-only axis could push sideways tire force into the
			-- terrain normal on slopes and create artificial vertical energy.
			local rightX = tangentY * normal[3] - tangentZ * normal[2]
			local rightY = tangentZ * normal[1] - tangentX * normal[3]
			local rightZ = tangentX * normal[2] - tangentY * normal[1]
			local rightLength = math.sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ)
			if rightLength > 1e-6 then
				rightX, rightY, rightZ = rightX / rightLength, rightY / rightLength, rightZ / rightLength
			else
				rightX, rightY, rightZ = wheelRightAxes[i][1], wheelRightAxes[i][2], wheelRightAxes[i][3]
			end
			wheelRightAxes[i] = { rightX, rightY, rightZ }
			local right = wheelRightAxes[i]
			local lateralVelocity = n.vx * right[1] + n.vy * right[2] + n.vz * right[3]
			local forwardVelocity = n.vx * tangentX + n.vy * tangentY + n.vz * tangentZ
			wheelForwardSpeeds[i] = forwardVelocity
			local drivenVelocityForSlip = if isDriven(i) and engineOn then d.wheel_speed * cfg.tire_radius else forwardVelocity
			local slipRatio = math.clamp((drivenVelocityForSlip - forwardVelocity) / math.max(math.abs(forwardVelocity), 0.25), -2, 2)
			local slipAngle = math.clamp(math.atan2(lateralVelocity, math.max(math.abs(forwardVelocity), 0.1)), -1.5, 1.5)
			local tire = self.tires[i]
			Tires.update(tire, self.tire_thermal, math.clamp(slipRatio, -0.35, 0.35), math.clamp(slipAngle, -0.35, 0.35), load, surfaceRoughness, 20, dt)
			local forces = Tires.calculatePacejkaForces(self.tire_params, slipRatio, slipAngle, load, terrainGrip * tire.grip_factor)
			wheelGrip[i] = math.max(forces.longitudinal_peak, forces.lateral_peak) / math.max(load, 1e-6)
			wheelLateral[i] = forces.lateral
			wheelLongCapacity[i] = math.max(forces.longitudinal_peak, 0)
		end
		wheelLoads[i] = load
		state.compression = compression
		state.load = load
		state.suspension_force = load
		state.angular_speed = wheelAngularSpeeds[i]
		state.is_airborne = airborne
		state.contact_point = result.contact_point
		state.previous_suspension_length = suspensionLength
	end

	-- Anti-roll bars transfer load without creating or destroying axle load.
	for _, axle in { { 1, 2 }, { 3, 4 } } do
		local left, right = axle[1], axle[2]
		if grounded[left] and grounded[right] then
			local travel = (self.suspension_config.wheels[left].travel + self.suspension_config.wheels[right].travel) * 0.5
			local transfer = self.suspension_config.anti_roll_bar_stiffness
				* (self.suspension_states[left].compression - self.suspension_states[right].compression) * math.max(travel, 0.02)
			transfer = math.clamp(transfer, -wheelLoads[right], wheelLoads[left])
			wheelLoads[left] = math.max(wheelLoads[left] - transfer, 0)
			wheelLoads[right] = math.max(wheelLoads[right] + transfer, 0)
			self.suspension_states[left].load = wheelLoads[left]
			self.suspension_states[right].load = wheelLoads[right]
		end
	end

	local driveTorque = d.last_drive_torque * vscTorqueModifier
	local torqueByWheel = { 0, 0, 0, 0 }
	if self.driveLayout == 1 then
		torqueByWheel[1], torqueByWheel[2] = Drivetrain.applyDifferential(
			d.differential, driveTorque, wheelGrip[1], wheelGrip[2], wheelAngularSpeeds[1], wheelAngularSpeeds[2])
	elseif self.driveLayout == 2 then
		local frontLeft, frontRight = Drivetrain.applyDifferential(d.differential, driveTorque * 0.5, wheelGrip[1], wheelGrip[2], wheelAngularSpeeds[1], wheelAngularSpeeds[2])
		local rearLeft, rearRight = Drivetrain.applyDifferential(d.differential, driveTorque * 0.5, wheelGrip[3], wheelGrip[4], wheelAngularSpeeds[3], wheelAngularSpeeds[4])
		torqueByWheel = { frontLeft, frontRight, rearLeft, rearRight }
	else
		torqueByWheel[3], torqueByWheel[4] = Drivetrain.applyDifferential(
			d.differential, driveTorque, wheelGrip[3], wheelGrip[4], wheelAngularSpeeds[3], wheelAngularSpeeds[4])
	end

	for i = 1, 4 do
		if grounded[i] and not nodes[i].fixed then
			local n = nodes[i]
			local normal = wheelNormals[i]
			local axis = wheelAxes[i]
			local right = wheelRightAxes[i]
			local load = wheelLoads[i]
			local capacity = math.sqrt(math.max((load * wheelGrip[i])^2 - wheelLateral[i]^2, 0))
			capacity = math.min(capacity, wheelLongCapacity[i])
			local requestedDrive = torqueByWheel[i] / math.max(self.suspension_config.wheels[i].tire_radius, 0.05)
			local driveForce = math.clamp(requestedDrive, -capacity, capacity)
			local brake = brakePressures[i] or 0
			if controls.handbrake and i >= 3 then brake = math.max(brake, 1) end
			local direction = Math.sign(wheelForwardSpeeds[i])
			if direction == 0 then direction = Math.sign(driveForce) end
			local brakeForce = brake * 10000
			local longitudinalForce = math.clamp(driveForce - direction * brakeForce, -capacity, capacity)
			local rollingForce = if math.abs(wheelForwardSpeeds[i]) > 0.1 then -direction * math.min(30 * wheelSurfaceRoughness[i], capacity) else 0
			local totalLongitudinal = longitudinalForce + rollingForce
			n.fx += wheelLateral[i] * right[1] + totalLongitudinal * axis[1] + load * normal[1]
			n.fy += wheelLateral[i] * right[2] + totalLongitudinal * axis[2] + load * normal[2]
			n.fz += wheelLateral[i] * right[3] + totalLongitudinal * axis[3] + load * normal[3]
		end
	end

	if self.fuel_tank then
		Suspension.updateFuel(self.fuel_tank, d.engine.rpm, d.throttle_input, d.engine.redline_rpm, dt)
	end
end
end

return Module
