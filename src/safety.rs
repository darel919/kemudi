use serde::{Deserialize, Serialize};

/// Safety system operating modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SafetyMode {
    Enabled,
    Sport,
    Disabled,
    Degraded,
    Faulted,
}

/// Four-channel ABS with individual wheel-speed sensors.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ABS {
    pub enabled: bool,
    pub mode: SafetyMode,
    /// Per-wheel brake pressure modifier (0=full release, 1=full pressure).
    pub wheel_pressure: [f64; 4],
    /// Slip target for each wheel.
    pub slip_threshold: f64,
    /// Valve response latency (seconds).
    pub valve_latency: f64,
    /// Pump response latency (seconds).
    pub pump_latency: f64,
    /// Currently modulating brake pressure.
    pub is_active: bool,
    /// Sensor health per wheel.
    pub sensor_ok: [bool; 4],
}

impl Default for ABS {
    fn default() -> Self {
        Self {
            enabled: true,
            mode: SafetyMode::Enabled,
            wheel_pressure: [1.0; 4],
            slip_threshold: 0.15,
            valve_latency: 0.01,
            pump_latency: 0.02,
            is_active: false,
            sensor_ok: [true; 4],
        }
    }
}

impl ABS {
    /// Update ABS based on wheel speeds and vehicle speed.
    /// Returns per-wheel brake pressure modifiers.
    pub fn update(
        &mut self,
        wheel_speeds: [f64; 4],
        vehicle_speed: f64,
        brake_command: f64,
        dt: f64,
    ) -> [f64; 4] {
        if !self.enabled || self.mode == SafetyMode::Disabled || self.mode == SafetyMode::Faulted {
            self.is_active = false;
            return [brake_command; 4];
        }

        self.is_active = false;
        for i in 0..4 {
            if !self.sensor_ok[i] {
                // Faulted sensor: apply fixed reduction
                self.wheel_pressure[i] = (brake_command * 0.6).max(0.0);
                continue;
            }

            let wheel_slip = if vehicle_speed > 0.5 {
                1.0 - (wheel_speeds[i] / vehicle_speed).clamp(0.0, 1.5)
            } else {
                0.0
            };

            // Sport mode: higher slip threshold
            let threshold = match self.mode {
                SafetyMode::Sport => self.slip_threshold * 1.5,
                _ => self.slip_threshold,
            };

            if wheel_slip > threshold && brake_command > 0.1 {
                // ABS intervention: reduce brake pressure for this wheel
                let reduction = ((wheel_slip - threshold) / threshold).min(1.0);
                self.wheel_pressure[i] = brake_command * (1.0 - reduction * 0.7);
                self.is_active = true;
            } else {
                // Rebuild pressure gradually
                let rebuild_rate = 2.0; // per second
                self.wheel_pressure[i] =
                    (self.wheel_pressure[i] + rebuild_rate * dt).min(brake_command);
            }
        }
        self.wheel_pressure
    }
}

/// Traction control: driven-wheel slip detection and intervention.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TractionControl {
    pub enabled: bool,
    pub mode: SafetyMode,
    /// Slip threshold before intervention.
    pub slip_threshold: f64,
    /// Maximum throttle reduction (0-1).
    pub throttle_reduction_max: f64,
    /// Whether brake-based torque vectoring is available.
    pub brake_intervention: bool,
    /// Current throttle modifier (0=full cut, 1=no reduction).
    pub throttle_modifier: f64,
    /// Currently intervening.
    pub is_active: bool,
}

impl Default for TractionControl {
    fn default() -> Self {
        Self {
            enabled: false,
            mode: SafetyMode::Enabled,
            slip_threshold: 0.12,
            throttle_reduction_max: 0.5,
            brake_intervention: false,
            throttle_modifier: 1.0,
            is_active: false,
        }
    }
}

impl TractionControl {
    /// Update traction control. Returns throttle modifier (multiply by throttle).
    pub fn update(
        &mut self,
        wheel_speeds: [f64; 4],
        vehicle_speed: f64,
        driven_wheels: &[usize],
        throttle_command: f64,
        _dt: f64,
    ) -> f64 {
        if !self.enabled || self.mode == SafetyMode::Disabled || self.mode == SafetyMode::Faulted {
            self.is_active = false;
            return 1.0;
        }

        self.is_active = false;
        self.throttle_modifier = 1.0;

        for &wheel in driven_wheels {
            let wheel_slip = if vehicle_speed > 0.5 {
                (wheel_speeds[wheel] / vehicle_speed - 1.0).max(0.0)
            } else {
                // At standstill the ground speed denominator is not useful.
                // Wheel speeds are already linear m/s, so compare the driven
                // wheel directly with the low-speed reference instead of
                // applying a dimensionally unrelated gravity conversion.
                (wheel_speeds[wheel] - vehicle_speed).max(0.0)
            };

            let threshold = match self.mode {
                SafetyMode::Sport => self.slip_threshold * 2.0,
                _ => self.slip_threshold,
            };

            if wheel_slip > threshold && throttle_command > 0.1 {
                let excess = (wheel_slip - threshold) / threshold;
                let reduction =
                    (excess * self.throttle_reduction_max).min(self.throttle_reduction_max);
                self.throttle_modifier = (1.0 - reduction).max(0.3);
                self.is_active = true;
            }
        }
        self.throttle_modifier
    }
}

/// Yaw stability control (VSC/ESC).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct YawStabilityControl {
    pub enabled: bool,
    pub mode: SafetyMode,
    /// Yaw rate sensor present.
    pub yaw_rate_sensor: bool,
    /// Lateral acceleration sensor present.
    pub lateral_accel_sensor: bool,
    /// Individual brake vectoring available.
    pub individual_brake: bool,
    /// Engine torque reduction available.
    pub torque_reduction: bool,
    /// Yaw error threshold (rad/s) before intervention.
    pub yaw_error_threshold: f64,
    /// Currently correcting.
    pub is_active: bool,
    /// Per-wheel braking torque from VSC (Nm).
    pub brake_torque: [f64; 4],
    /// Engine torque reduction factor (0-1).
    pub torque_modifier: f64,
}

impl Default for YawStabilityControl {
    fn default() -> Self {
        Self {
            enabled: false,
            mode: SafetyMode::Enabled,
            yaw_rate_sensor: true,
            lateral_accel_sensor: true,
            individual_brake: true,
            torque_reduction: true,
            yaw_error_threshold: 0.12,
            is_active: false,
            brake_torque: [0.0; 4],
            torque_modifier: 1.0,
        }
    }
}

impl YawStabilityControl {
    /// Update VSC/ESC. Returns (per-wheel brake torques, torque modifier).
    pub fn update(
        &mut self,
        actual_yaw_rate: f64,
        desired_yaw_rate: f64,
        steering_angle: f64,
        _lateral_accel: f64,
        vehicle_speed: f64,
        _wheel_speeds: [f64; 4],
        _dt: f64,
    ) -> ([f64; 4], f64) {
        if !self.enabled || self.mode == SafetyMode::Disabled || self.mode == SafetyMode::Faulted {
            self.is_active = false;
            return ([0.0; 4], 1.0);
        }

        self.brake_torque = [0.0; 4];
        self.torque_modifier = 1.0;
        self.is_active = false;

        // Yaw error
        let yaw_error = actual_yaw_rate - desired_yaw_rate;

        let threshold = match self.mode {
            SafetyMode::Sport => self.yaw_error_threshold * 1.5,
            _ => self.yaw_error_threshold,
        };

        if yaw_error.abs() > threshold && vehicle_speed > 5.0 {
            self.is_active = true;

            if self.individual_brake {
                // Compare yaw error in the driver's turn direction. A left
                // turn has a negative desired yaw rate, so raw error sign
                // alone would misclassify left-turn oversteer as understeer.
                let turn_sign = if steering_angle.abs() > 1e-6 {
                    steering_angle.signum()
                } else if desired_yaw_rate.abs() > 1e-6 {
                    desired_yaw_rate.signum()
                } else {
                    1.0
                };
                let signed_yaw_error = yaw_error * turn_sign;

                // Oversteer: brake the outer front.
                // Understeer: brake the inner rear.
                if signed_yaw_error > 0.0 {
                    // Positive steering is a right turn, so the outer front is
                    // front-left. Negative steering makes front-right outer.
                    if steering_angle > 0.0 {
                        self.brake_torque[0] = yaw_error.abs() * 15.0; // front-left
                    } else {
                        self.brake_torque[1] = yaw_error.abs() * 15.0; // front-right
                    }
                } else {
                    // The inner rear is rear-right for a right turn and
                    // rear-left for a left turn.
                    if steering_angle > 0.0 {
                        self.brake_torque[3] = yaw_error.abs() * 10.0; // rear-right
                    } else {
                        self.brake_torque[2] = yaw_error.abs() * 10.0; // rear-left
                    }
                }
            }

            if self.torque_reduction {
                // ESC should trim excess yaw, not remove half the engine
                // torque during an ordinary steering transient. The old
                // gain made a normal launch oscillate between understeer and
                // oversteer, then the corrective brake could turn the car
                // past the driver's requested direction.
                let reduction = (yaw_error.abs() / threshold * 0.12).min(0.25);
                self.torque_modifier = 1.0 - reduction;
            }
        }

        (self.brake_torque, self.torque_modifier)
    }
}

/// Sensor health status for ADAS.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SensorFault {
    None,
    CameraBlocked,
    RadarFault,
    LidarFault,
    WiringFault,
    CalibError,
}

/// ADAS detection state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ADASDetection {
    pub detected: bool,
    pub distance: f64,
    pub relative_speed: f64,
    pub confidence: f64,
    pub time_to_collision: f64,
}

/// Advanced Driver Assistance Systems.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ADAS {
    pub forward_collision_warning: bool,
    pub automatic_emergency_braking: bool,
    pub adaptive_cruise_control: bool,
    pub blind_spot_warning: bool,
    pub lane_departure_warning: bool,
    pub lane_departure_assist: bool,
    pub parking_sensors: bool,
    /// Maximum detection range (meters).
    pub perception_range: f64,
    /// Field of view (degrees).
    pub radar_fov: f64,
    pub camera_fov: f64,
    /// Current sensor faults.
    pub sensor_faults: SensorFault,
    /// Forward collision detection.
    pub front_target: ADASDetection,
    /// Blind spot detection (left, right).
    pub blind_spot: [ADASDetection; 2],
    /// Current AEB intervention.
    pub aeb_active: bool,
    /// ACC target speed (km/h).
    pub acc_target_speed: f64,
    /// Lane departure warning active.
    pub ldw_active: bool,
    /// Braking authority (0-1).
    pub braking_authority: f64,
}

impl Default for ADAS {
    fn default() -> Self {
        Self {
            forward_collision_warning: false,
            automatic_emergency_braking: false,
            adaptive_cruise_control: false,
            blind_spot_warning: false,
            lane_departure_warning: false,
            lane_departure_assist: false,
            parking_sensors: false,
            perception_range: 200.0,
            radar_fov: 20.0,
            camera_fov: 50.0,
            sensor_faults: SensorFault::None,
            front_target: ADASDetection {
                detected: false,
                distance: 0.0,
                relative_speed: 0.0,
                confidence: 0.0,
                time_to_collision: 0.0,
            },
            blind_spot: [
                ADASDetection {
                    detected: false,
                    distance: 0.0,
                    relative_speed: 0.0,
                    confidence: 0.0,
                    time_to_collision: 0.0,
                },
                ADASDetection {
                    detected: false,
                    distance: 0.0,
                    relative_speed: 0.0,
                    confidence: 0.0,
                    time_to_collision: 0.0,
                },
            ],
            aeb_active: false,
            acc_target_speed: 0.0,
            ldw_active: false,
            braking_authority: 0.7,
        }
    }
}

impl ADAS {
    /// Update forward collision detection and AEB.
    pub fn update_forward_collision(
        &mut self,
        target_distance: f64,
        target_relative_speed: f64,
        vehicle_speed: f64,
        _dt: f64,
    ) {
        if self.sensor_faults != SensorFault::None {
            self.front_target.detected = false;
            self.aeb_active = false;
            return;
        }

        // Perception range and confidence
        let in_range = target_distance > 0.0 && target_distance < self.perception_range;
        let speed_factor = (vehicle_speed / 30.0).min(1.0);
        let confidence = if in_range { 0.8 * speed_factor } else { 0.0 };

        self.front_target = ADASDetection {
            detected: in_range,
            distance: target_distance,
            relative_speed: target_relative_speed,
            confidence,
            time_to_collision: if target_relative_speed < 0.0 {
                target_distance / (-target_relative_speed + 0.1)
            } else {
                f64::INFINITY
            },
        };

        // FCW: warn if TTC < 2.5s
        self.forward_collision_warning = self.front_target.detected
            && self.front_target.time_to_collision < 2.5
            && self.front_target.confidence > 0.3;

        // AEB: brake if TTC < 1.0s and confidence high
        self.aeb_active = self.automatic_emergency_braking
            && self.front_target.detected
            && self.front_target.time_to_collision < 1.0
            && self.front_target.confidence > 0.5;
    }

    /// Get AEB brake command (0-1). Returns 0 if AEB not active.
    pub fn get_aeb_brake(&self, brake_authority: f64) -> f64 {
        if self.aeb_active {
            // Progressive braking based on TTC
            let ttc = self.front_target.time_to_collision;
            let urgency = (1.0 - ttc).clamp(0.0, 1.0);
            urgency * self.braking_authority * brake_authority
        } else {
            0.0
        }
    }

    /// Update blind spot detection.
    pub fn update_blind_spot(
        &mut self,
        left_distance: f64,
        right_distance: f64,
        _vehicle_speed: f64,
    ) {
        if self.sensor_faults != SensorFault::None || !self.blind_spot_warning {
            self.blind_spot[0].detected = false;
            self.blind_spot[1].detected = false;
            return;
        }

        // Blind spot: detect objects within 5m laterally and 0-10m behind
        self.blind_spot[0] = ADASDetection {
            detected: left_distance > 0.0 && left_distance < 10.0,
            distance: left_distance,
            relative_speed: 0.0,
            confidence: if left_distance > 0.0 && left_distance < 10.0 {
                0.7
            } else {
                0.0
            },
            time_to_collision: f64::INFINITY,
        };
        self.blind_spot[1] = ADASDetection {
            detected: right_distance > 0.0 && right_distance < 10.0,
            distance: right_distance,
            relative_speed: 0.0,
            confidence: if right_distance > 0.0 && right_distance < 10.0 {
                0.7
            } else {
                0.0
            },
            time_to_collision: f64::INFINITY,
        };
    }
}

/// Combined safety system state for telemetry and UI.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetySystemState {
    pub abs: ABS,
    pub traction_control: TractionControl,
    pub vsc_esc: YawStabilityControl,
    pub adas: ADAS,
}

impl Default for SafetySystemState {
    fn default() -> Self {
        Self {
            abs: ABS::default(),
            traction_control: TractionControl::default(),
            vsc_esc: YawStabilityControl::default(),
            adas: ADAS::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_abs_reduces_brake_on_lockup() {
        let mut abs = ABS::default();
        // Front-left wheel locking up (speed=0 while vehicle moving)
        let speeds = [0.0, 20.0, 20.0, 20.0];
        let result = abs.update(speeds, 20.0, 1.0, 0.016);
        assert!(
            result[0] < 1.0,
            "Locked wheel should have reduced brake pressure"
        );
        assert!(abs.is_active, "ABS should be active during lockup");
    }

    #[test]
    fn test_abs_no_intervention_when_no_slip() {
        let mut abs = ABS::default();
        let speeds = [20.0, 20.0, 20.0, 20.0];
        let result = abs.update(speeds, 20.0, 0.5, 0.016);
        assert_eq!(
            result, [0.5; 4],
            "No slip should pass through brake command unchanged"
        );
        assert!(!abs.is_active);
    }

    #[test]
    fn test_abs_disabled_passes_through() {
        let mut abs = ABS::default();
        abs.mode = SafetyMode::Disabled;
        let speeds = [0.0, 20.0, 20.0, 20.0];
        let result = abs.update(speeds, 20.0, 1.0, 0.016);
        assert_eq!(result, [1.0; 4], "Disabled ABS should pass through");
    }

    #[test]
    fn test_abs_faulted_sensor_reduces_pressure() {
        let mut abs = ABS::default();
        abs.sensor_ok[0] = false;
        let speeds = [20.0, 20.0, 20.0, 20.0];
        let result = abs.update(speeds, 20.0, 1.0, 0.016);
        assert!(
            result[0] < 1.0,
            "Faulted sensor should reduce brake pressure"
        );
    }

    #[test]
    fn test_traction_control_reduces_throttle_on_spin() {
        let mut tc = TractionControl::default();
        tc.enabled = true;
        // Rear wheels spinning faster than vehicle
        let speeds = [20.0, 20.0, 30.0, 30.0];
        let modifier = tc.update(speeds, 20.0, &[2, 3], 1.0, 0.016);
        assert!(modifier < 1.0, "Spinning wheels should reduce throttle");
        assert!(tc.is_active);
    }

    #[test]
    fn test_traction_control_disabled() {
        let mut tc = TractionControl::default();
        // Not enabled
        let speeds = [20.0, 20.0, 40.0, 40.0];
        let modifier = tc.update(speeds, 20.0, &[2, 3], 1.0, 0.016);
        assert_eq!(modifier, 1.0, "Disabled TC should not intervene");
    }

    #[test]
    fn test_vsc_corrects_oversteer() {
        let mut vsc = YawStabilityControl::default();
        vsc.enabled = true;
        // Oversteer: actual yaw > desired yaw, turning right
        let (brakes, torque_mod) = vsc.update(15.0, 5.0, 0.3, 5.0, 30.0, [20.0; 4], 0.016);
        assert!(vsc.is_active, "VSC should detect oversteer");
        assert!(
            brakes.iter().any(|&b| b > 0.0),
            "Should apply braking torque"
        );
        assert!(torque_mod < 1.0, "Should reduce engine torque");
    }

    #[test]
    fn test_vsc_selects_outer_front_and_inner_rear_by_turn_direction() {
        let mut vsc = YawStabilityControl::default();
        vsc.enabled = true;
        let (right_oversteer, _) = vsc.update(1.0, 0.0, 0.3, 0.0, 20.0, [20.0; 4], 0.016);
        assert!(
            right_oversteer[0] > 0.0,
            "right-turn oversteer brakes outer front-left"
        );
        assert_eq!(right_oversteer[1], 0.0);

        let (left_oversteer, _) = vsc.update(-1.0, 0.0, -0.3, 0.0, 20.0, [20.0; 4], 0.016);
        assert!(
            left_oversteer[1] > 0.0,
            "left-turn oversteer brakes outer front-right"
        );
        assert_eq!(left_oversteer[0], 0.0);

        let (right_understeer, _) = vsc.update(0.0, 1.0, 0.3, 0.0, 20.0, [20.0; 4], 0.016);
        assert!(
            right_understeer[3] > 0.0,
            "right-turn understeer brakes inner rear-right"
        );
        assert_eq!(right_understeer[2], 0.0);
    }

    #[test]
    fn test_vsc_no_intervention_at_low_speed() {
        let mut vsc = YawStabilityControl::default();
        vsc.enabled = true;
        let (brakes, _) = vsc.update(20.0, 5.0, 0.3, 5.0, 3.0, [5.0; 4], 0.016);
        assert!(!vsc.is_active, "VSC should not intervene at low speed");
        assert_eq!(brakes, [0.0; 4]);
    }

    #[test]
    fn test_adas_aeb_activates_on_close_target() {
        let mut adas = ADAS::default();
        adas.automatic_emergency_braking = true;
        adas.update_forward_collision(5.0, -15.0, 30.0, 0.1);
        assert!(adas.aeb_active, "AEB should activate with close TTC");
        let brake = adas.get_aeb_brake(1.0);
        assert!(brake > 0.0, "AEB should produce brake command");
    }

    #[test]
    fn test_adas_no_aeb_when_no_target() {
        let mut adas = ADAS::default();
        adas.automatic_emergency_braking = true;
        adas.update_forward_collision(0.0, 0.0, 30.0, 0.1);
        assert!(!adas.aeb_active);
    }

    #[test]
    fn test_adas_fcw_warns_before_aeb() {
        let mut adas = ADAS::default();
        adas.automatic_emergency_braking = true;
        // Close target: FCW (TTC ~2.0s) but no AEB (TTC > 1.0s)
        adas.update_forward_collision(20.0, -10.0, 30.0, 0.1);
        assert!(
            adas.forward_collision_warning,
            "FCW should warn at TTC < 2.5s"
        );
        assert!(!adas.aeb_active, "AEB should not activate at TTC > 1.0s");
    }

    #[test]
    fn test_adas_sensor_fault_disables() {
        let mut adas = ADAS::default();
        adas.automatic_emergency_braking = true;
        adas.sensor_faults = SensorFault::CameraBlocked;
        adas.update_forward_collision(5.0, -15.0, 30.0, 0.1);
        assert!(!adas.aeb_active, "Sensor fault should disable AEB");
    }

    #[test]
    fn test_adas_blind_spot_detection() {
        let mut adas = ADAS::default();
        adas.blind_spot_warning = true;
        adas.update_blind_spot(3.0, 15.0, 30.0);
        assert!(
            adas.blind_spot[0].detected,
            "Left blind spot should detect close object"
        );
        assert!(
            !adas.blind_spot[1].detected,
            "Right blind spot should not detect far object"
        );
    }

    #[test]
    fn test_adas_ttc_calculation() {
        let mut adas = ADAS::default();
        adas.update_forward_collision(100.0, -50.0, 30.0, 0.1);
        // TTC = distance / closing_speed = 100 / 50 = 2.0
        assert!(
            (adas.front_target.time_to_collision - 2.0).abs() < 0.1,
            "TTC should be ~2.0s, got {}",
            adas.front_target.time_to_collision
        );
    }
}
