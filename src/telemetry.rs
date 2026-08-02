use crate::drivetrain;
use crate::types::*;

impl PhysicsWorld {
    pub(crate) fn update_telemetry(&mut self) {
        let count = self.nodes.iter().filter(|n| !n.fixed).count().max(1) as f64;
        let vx = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.vx)
            .sum::<f64>()
            / count;
        let vz = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.vz)
            .sum::<f64>()
            / count;
        let speed = (vx * vx + vz * vz).sqrt();
        self.telemetry[T_SPEED_MPS] = speed;
        self.telemetry[T_SPEED_KMH] = speed * 3.6;
        self.telemetry[T_RPM] = if self.controls.engine_on {
            self.drivetrain.get_rpm()
        } else {
            0.0
        };
        self.telemetry[T_GEAR] = self.drivetrain.get_gear() as f64;
        self.telemetry[T_THROTTLE] = self.controls.throttle;
        self.telemetry[T_BRAKE] = self.controls.brake;
        self.telemetry[T_STEERING] = self.steering_angle;
        self.telemetry[T_CLUTCH] = self.drivetrain.get_clutch();
        self.telemetry[T_WHEEL_SPEED] = self.drivetrain.wheel_speed;
        self.telemetry[T_DRIVE_TORQUE] = self.drivetrain.get_drive_torque();
        self.telemetry[T_ENGINE_RUNNING] = if self.controls.engine_on
            && self.fuel.current_level > 0.0
            && !self.engine_damage.is_seized
        {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_SHIFT_PHASE] = match self.drivetrain.shift_phase {
            drivetrain::ShiftPhase::Idle => 0.0,
            drivetrain::ShiftPhase::Disengaging => 1.0,
            drivetrain::ShiftPhase::Neutral => 2.0,
            drivetrain::ShiftPhase::Engaging => 3.0,
        };
        self.telemetry[T_ABS_ACTIVE] = if self.safety.abs.is_active { 1.0 } else { 0.0 };
        self.telemetry[T_TC_ACTIVE] = if self.safety.traction_control.is_active {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_VSC_ACTIVE] = if self.safety.vsc_esc.is_active {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_TCM_STATE] = self.tcm.state as u8 as f64;
        self.telemetry[T_TCM_LINE_PRESSURE] = self.tcm.line_pressure;
        self.telemetry[T_TCM_LOCKUP] = if self.tcm.converter_lockup { 1.0 } else { 0.0 };
        self.telemetry[T_REQUESTED_GEAR] = self.drivetrain.pending_gear as f64;
        self.telemetry[T_INPUT_SHAFT_RPM] = self.drivetrain.engine.rpm;
        self.telemetry[T_OUTPUT_SHAFT_RPM] =
            self.drivetrain.wheel_speed.abs() * 60.0 / (2.0 * std::f64::consts::PI);
        self.telemetry[T_CONVERTER_COUPLING] = self.drivetrain.converter_coupling;
        self.telemetry[T_TCM_FAULT_MASK] = self.tcm.faults.fault_mask as f64;
        self.telemetry[T_TCM_DIAGNOSTIC_CODE] = self.tcm.last_diagnostic_code as f64;
        self.telemetry[T_TCM_SENSOR_INPUT_RPM] = self.tcm.observed_input_rpm;
        self.telemetry[T_TCM_SENSOR_SPEED_KMH] = self.tcm.observed_output_speed;
        self.telemetry[T_TCM_SENSOR_AGE] = self.tcm.sensor_age;
        self.telemetry[T_TCM_SHIFT_LATENCY] = self.tcm.shift_latency;
        self.telemetry[T_TCM_TORQUE_REDUCTION] = self.tcm.torque_reduction_request;
        self.telemetry[T_TCM_FAIL_SAFE_GEAR] = self
            .tcm
            .fail_safe_gear(self.drivetrain.transmission.gear_ratios.len() as i32)
            as f64;
        self.telemetry[T_BROKEN_BEAMS] = self.beams.iter().filter(|b| b.broken).count() as f64;
        let structural_damage = self.telemetry[T_BROKEN_BEAMS] / self.beams.len().max(1) as f64;
        let engine_damage = self
            .engine_damage
            .wear_level
            .max(self.engine_damage.bearing_damage);
        self.telemetry[T_DAMAGE] = structural_damage
            .max(engine_damage)
            .max(self.body_damage)
            .clamp(0.0, 1.0);
        self.telemetry[T_ENGINE_WARNING] = if self.telemetry[T_COOLANT] > 105.0
            || (self.telemetry[T_ENGINE_RUNNING] > 0.5 && self.telemetry[T_OIL_PRESSURE] < 80.0)
            || self.telemetry[T_ENGINE_STAGE] > 0.0
        {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_ADAS_FCW] = if self.safety.adas.forward_collision_warning {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_ADAS_AEB] = if self.safety.adas.aeb_active {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_ADAS_CONFIDENCE] = self.safety.adas.front_target.confidence;
        self.telemetry[T_TIRE_WEAR] = self.tires.iter().map(|t| t.wear).sum::<f64>() / 4.0;
        self.telemetry[T_TIRE_TEMP] = self.tires.iter().map(|t| t.temperature).sum::<f64>() / 4.0;
        for i in 0..4 {
            self.telemetry[T_WHEEL_SPEED_FL + i] =
                self.wheels[i].angular_speed.abs() * self.suspension.wheels[i].tire_radius * 3.6;
            self.telemetry[T_SUSPENSION_COMPRESSION_FL + i] = self.wheels[i].compression * 100.0;
            self.telemetry[T_SUSPENSION_LOAD_FL + i] = self.wheels[i].load;
            self.telemetry[T_TIRE_TEMP_FL + i] = self.tires[i].temperature;
            self.telemetry[T_TIRE_WEAR_FL + i] = self.tires[i].wear;
        }
        let deformation_depth = self
            .beams
            .iter()
            .map(|beam| (beam.length - beam.initial_length).abs())
            .fold(0.0, f64::max);
        self.telemetry[T_IMPACT_SEVERITY] = self.impact_severity;
        self.telemetry[T_DEFORMATION_DEPTH] = deformation_depth;
        self.telemetry[T_BROKEN_PARTS] = self.telemetry[T_BROKEN_BEAMS];
        self.telemetry[T_DAMAGE_ZONE] = self.damage_zone as f64;
        self.telemetry[T_SIM_TIME] = self.time;
    }
}
