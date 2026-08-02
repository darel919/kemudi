local Module = {}

function Module.install(World, Contract, Safety)
-- Telemetry update
function World:_updateTelemetry()
	local t = self.telemetry
	local nodes = self.nodes
	local count = #nodes
	-- Speed from average vehicle velocity
	local avgVx, avgVz = 0, 0
	local dynCount = 0
	for i = 1, count do
		local n = nodes[i]
		if not n.fixed then
			avgVx += n.vx
			avgVz += n.vz
			dynCount += 1
		end
	end
	if dynCount > 0 then
		avgVx /= dynCount
		avgVz /= dynCount
	end
	local speed = math.sqrt(avgVx * avgVx + avgVz * avgVz)
	t[Contract.T.SPEED_MPS + 1] = speed
	t[Contract.T.SPEED_KMH + 1] = speed * 3.6
	-- Sim time
	t[Contract.T.SIM_TIME + 1] = self.time
	-- Broken beams count
	local broken = 0
	for _, beam in self.beams do
		if beam.broken then broken += 1 end
	end
	t[Contract.T.BROKEN_BEAMS + 1] = broken

	if self.drivetrain then
		local d = self.drivetrain
		t[Contract.T.RPM + 1] = d.engine.rpm
		t[Contract.T.GEAR + 1] = d.transmission.current_gear
		t[Contract.T.THROTTLE + 1] = d.throttle_input
		t[Contract.T.BRAKE + 1] = d.brake_input
		t[Contract.T.STEERING + 1] = self.steering_angle
		t[Contract.T.CLUTCH + 1] = d.transmission.clutch_engagement
		t[Contract.T.WHEEL_SPEED + 1] = d.wheel_speed
		t[Contract.T.DRIVE_TORQUE + 1] = d.last_drive_torque
		t[Contract.T.ENGINE_RUNNING + 1] = if d.engine.rpm > 0 then 1 else 0
		t[Contract.T.ENGINE_STAGE + 1] = self.controls.engine_stage or 0
		t[Contract.T.SHIFT_PHASE + 1] = if d.shift.phase == "Idle" then 0 elseif d.shift.phase == "Disengaging" then 1 elseif d.shift.phase == "Neutral" then 2 else 3
		if d.tcm then
			local tcmState = ({ Normal = 0, Sport = 1, Winter = 2, Manual = 3, Limp = 4, Fault = 5, Off = 6 })[d.tcm.state] or 0
			t[Contract.T.TCM_STATE + 1] = tcmState
			t[Contract.T.TCM_LINE_PRESSURE + 1] = d.tcm.line_pressure
			t[Contract.T.TCM_LOCKUP + 1] = if d.tcm.converter_lockup then 1 else 0
			t[Contract.T.TCM_FAULT_MASK + 1] = d.tcm.fault_mask
			t[Contract.T.TCM_DIAGNOSTIC_CODE + 1] = d.tcm.last_diagnostic_code
			t[Contract.T.TCM_SENSOR_INPUT_RPM + 1] = d.tcm.observed_input_rpm
			t[Contract.T.TCM_SENSOR_SPEED_KMH + 1] = d.tcm.observed_output_speed
			t[Contract.T.TCM_SENSOR_AGE + 1] = d.tcm.sensor_age
			t[Contract.T.TCM_SHIFT_LATENCY + 1] = d.tcm.shift_latency
			t[Contract.T.TCM_TORQUE_REDUCTION + 1] = d.tcm.torque_reduction_request
			t[Contract.T.TCM_FAIL_SAFE_GEAR + 1] = math.clamp(d.tcm.limp_gear, 1, math.max(#d.transmission.gear_ratios, 1))
			t[Contract.T.REQUESTED_GEAR + 1] = d.tcm.pending_gear or d.transmission.current_gear
			t[Contract.T.INPUT_SHAFT_RPM + 1] = d.tcm.observed_input_rpm
			local outputWheelSpeed = if d.tcm.wheel_radius > 0
				then (d.tcm.observed_output_speed / 3.6) / d.tcm.wheel_radius
				else 0
			local outputRatio = d.transmission.gear_ratios[d.transmission.current_gear] or 0
			t[Contract.T.OUTPUT_SHAFT_RPM + 1] = math.abs(outputWheelSpeed * outputRatio * d.transmission.final_drive) * 60 / (2 * math.pi)
			t[Contract.T.CONVERTER_COUPLING + 1] = d.converter_coupling
		end
	end

	local totalLoad, totalGrip, contactCount, terrainSum = 0, 0, 0, 0
	if self.suspension_states then
		for i = 1, math.min(#self.suspension_states, 4) do
			local wheel = self.suspension_states[i]
			totalLoad += wheel.load
			if not wheel.is_airborne then contactCount += 1 end
			local compressionIndex = Contract.T.SUSP_COMPRESSION_FL + i - 1
			local loadIndex = Contract.T.SUSP_LOAD_FL + i - 1
			t[compressionIndex + 1] = wheel.compression
			t[loadIndex + 1] = wheel.load
			t[Contract.T.WHEEL_SPEED_FL + i] = wheel.angular_speed
			terrainSum += wheel.contact_point[2]
			if self.tires and self.tires[i] then
				t[Contract.T.TIRE_TEMP_FL + i - 1 + 1] = self.tires[i].temperature
				t[Contract.T.TIRE_WEAR_FL + i - 1 + 1] = self.tires[i].wear
				totalGrip += self.tires[i].grip_factor
			end
		end
	end
	t[Contract.T.SUSPENSION_LOAD + 1] = totalLoad
	t[Contract.T.CONTACTS + 1] = contactCount
	t[Contract.T.TERRAIN_HEIGHT + 1] = if contactCount > 0 then terrainSum / contactCount else 0
	local wheelCount = if self.suspension_states then math.min(#self.suspension_states, 4) else 0
	t[Contract.T.GRIP + 1] = totalGrip / math.max(wheelCount, 1)
	if self.fuel_tank then
		t[Contract.T.FUEL + 1] = self.fuel_tank.current_level
		t[Contract.T.FUEL_PERCENT + 1] = self.fuel_tank.current_level / math.max(self.fuel_tank.max_capacity, 1)
	end
	if self.safety then
		t[Contract.T.ABS_ACTIVE + 1] = if self.safety.abs.is_active then 1 else 0
		t[Contract.T.TC_ACTIVE + 1] = if self.safety.traction_control.is_active then 1 else 0
		t[Contract.T.VSC_ACTIVE + 1] = if self.safety.vsc.is_active then 1 else 0
		t[Contract.T.YAW_ERROR + 1] = self.yawRate
		t[Contract.T.ADAS_FCW + 1] = if self.safety.adas.forward_collision_warning then 1 else 0
		t[Contract.T.ADAS_AEB + 1] = if self.safety.adas.aeb_active then 1 else 0
		t[Contract.T.ADAS_CONFIDENCE + 1] = self.safety.adas.front_target.confidence
		t[Contract.T.ADAS_TARGET_DETECTED + 1] = if self.safety.adas.front_target.detected then 1 else 0
		t[Contract.T.ADAS_TARGET_DISTANCE + 1] = self.safety.adas.front_target.distance
		t[Contract.T.ADAS_TIME_TO_COLLISION + 1] = self.safety.adas.front_target.time_to_collision
		t[Contract.T.ADAS_RELATIVE_SPEED + 1] = self.safety.adas.front_target.relative_speed
		t[Contract.T.ADAS_BRAKE_COMMAND + 1] = Safety.getAEBBrake(self.safety.adas, 1.0)
	end
end
end

return Module
