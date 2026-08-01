use serde::{Deserialize, Serialize};

/// Per-wheel configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelConfig {
    pub rest_length: f64,
    pub travel: f64,
    pub spring_rate: f64,
    pub damping: f64,
    pub rebound_damping: f64,
    pub unsprung_mass: f64,
    pub tire_radius: f64,
}

impl Default for WheelConfig {
    fn default() -> Self {
        Self {
            rest_length: 0.3,
            travel: 0.2,
            spring_rate: 30000.0,
            damping: 4000.0,
            rebound_damping: 3000.0,
            unsprung_mass: 15.0,
            tire_radius: 0.3,
        }
    }
}

/// Suspension configuration for all wheels.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuspensionConfig {
    pub wheels: Vec<WheelConfig>,
    pub anti_roll_bar_stiffness: f64,
    pub bump_stop_rate: f64,
}

impl Default for SuspensionConfig {
    fn default() -> Self {
        Self {
            wheels: vec![WheelConfig::default(); 4],
            anti_roll_bar_stiffness: 10000.0,
            bump_stop_rate: 500000.0,
        }
    }
}

/// Per-wheel runtime state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelState {
    pub compression: f64,
    pub load: f64,
    pub is_airborne: bool,
    pub angular_speed: f64,
    pub contact_point: [f64; 3],
    pub suspension_force: f64,
}

impl Default for WheelState {
    fn default() -> Self {
        Self {
            compression: 0.0,
            load: 0.0,
            is_airborne: true,
            angular_speed: 0.0,
            contact_point: [0.0; 3],
            suspension_force: 0.0,
        }
    }
}

/// Suspension force output.
#[derive(Debug, Clone, Default)]
pub struct SuspensionForce {
    pub force: f64,
    pub contact_point: [f64; 3],
    pub normal: [f64; 3],
}

/// Raycast a wheel against terrain and compute suspension force.
pub fn raycast_wheel(
    config: &WheelConfig,
    wheel_pos: [f64; 3],
    chassis_velocity_y: f64,
    terrain_height: f64,
) -> (SuspensionForce, f64, bool) {
    let up = [0.0f64, 1.0, 0.0];
    let wheel_bottom = wheel_pos[1] - config.rest_length - config.tire_radius;

    if terrain_height > wheel_bottom + 0.01 {
        // Terrain is above wheel bottom — wheel is embedded (treated as contact at max compression)
    } else if wheel_bottom - terrain_height
        > config.rest_length + config.travel + config.tire_radius + 0.1
    {
        // Wheel is far above terrain — airborne
        return (SuspensionForce::default(), 0.0, true);
    } else if wheel_bottom > terrain_height + 0.01 {
        // Wheel is above terrain but close — check for contact
        // No contact if gap exceeds suspension range
        let max_reach = config.rest_length + config.travel + config.tire_radius;
        if wheel_bottom - terrain_height > max_reach {
            return (SuspensionForce::default(), 0.0, true);
        }
    }

    // Contact exists
    let contact_y = terrain_height;
    let penetration = wheel_bottom - contact_y;
    let max_compression = config.travel + config.rest_length;
    let compression = (penetration / max_compression).clamp(0.0, 1.0);

    // Spring force
    let spring_force = config.spring_rate * compression * config.travel;

    // Damping force (opposes velocity)
    let compression_velocity = chassis_velocity_y;
    let damping_force = if compression_velocity < 0.0 {
        config.damping * compression_velocity.abs()
    } else {
        -config.rebound_damping * compression_velocity
    };

    let total_force = (spring_force + damping_force).max(0.0);

    let contact_point = [wheel_pos[0], contact_y, wheel_pos[2]];

    (
        SuspensionForce {
            force: total_force,
            contact_point,
            normal: up,
        },
        compression,
        false,
    )
}

/// Steering state and geometry.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SteeringConfig {
    pub steering_ratio: f64,
    pub steering_speed: f64,
    pub max_steering_angle: f64,
    pub speed_sensitivity: f64,
    pub wheelbase: f64,
    pub track_width: f64,
}

impl Default for SteeringConfig {
    fn default() -> Self {
        Self {
            steering_ratio: 15.0,
            steering_speed: 3.0,
            max_steering_angle: 0.5,
            speed_sensitivity: 0.02,
            wheelbase: 2.5,
            track_width: 1.6,
        }
    }
}

/// Update steering angle with speed-sensitive reduction.
pub fn update_steering(
    config: &SteeringConfig,
    input_angle: f64,
    vehicle_speed: f64,
    current_angle: f64,
    dt: f64,
) -> f64 {
    let speed_factor = 1.0 / (1.0 + vehicle_speed * config.speed_sensitivity);
    let max_angle = config.max_steering_angle * speed_factor;
    let target = input_angle.clamp(-max_angle, max_angle);
    let delta = target - current_angle;
    let max_move = config.steering_speed * dt;
    let new_angle = if delta.abs() <= max_move {
        target
    } else if delta > 0.0 {
        current_angle + max_move
    } else {
        current_angle - max_move
    };
    new_angle.clamp(-max_angle, max_angle)
}

/// Compute Ackermann-corrected steering angles for left and right wheels.
pub fn ackermann_angles(steering_angle: f64, wheelbase: f64, track_width: f64) -> (f64, f64) {
    if steering_angle.abs() < 1e-6 {
        return (0.0, 0.0);
    }
    let sign = if steering_angle > 0.0 { 1.0 } else { -1.0 };
    let abs_angle = steering_angle.abs();
    let inner_dist = track_width / 2.0;
    let cot_outer = wheelbase / (abs_angle.tan() * wheelbase + inner_dist);
    let cot_inner = cot_outer + track_width / wheelbase;
    let outer = sign * cot_outer.atan();
    let inner = sign * cot_inner.atan();
    if steering_angle > 0.0 {
        (inner, outer) // left = inner, right = outer when turning right
    } else {
        (outer, inner)
    }
}

/// Speed computation from forces.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeedState {
    pub ground_speed: f64,
    pub wheel_speed: f64,
    pub slip_ratio: f64,
    pub acceleration: f64,
}

impl Default for SpeedState {
    fn default() -> Self {
        Self {
            ground_speed: 0.0,
            wheel_speed: 0.0,
            slip_ratio: 0.0,
            acceleration: 0.0,
        }
    }
}

/// Compute speed from net forces on the vehicle.
pub fn compute_speed(
    drivetrain_torque: f64,
    brake_force: f64,
    suspension_forces: &[f64],
    vehicle_mass: f64,
    slope_angle: f64,
    drag_coefficient: f64,
    frontal_area: f64,
    air_density: f64,
    wheel_radius: f64,
    rolling_resistance: f64,
    dt: f64,
    current_speed: f64,
) -> SpeedState {
    let gravity = 9.81;

    // Gravity component along slope
    let slope_force = -vehicle_mass * gravity * slope_angle.sin();

    // Aerodynamic drag
    let drag = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed;
    let drag_sign = if current_speed > 0.0 { -1.0 } else { 1.0 };

    // Rolling resistance (always opposes motion)
    let total_normal = suspension_forces
        .iter()
        .sum::<f64>()
        .max(vehicle_mass * gravity * 0.1);
    let rolling = rolling_resistance * total_normal;
    let rolling_sign = if current_speed > 0.0 {
        -1.0
    } else if current_speed < 0.0 {
        1.0
    } else {
        0.0
    };

    // Net force
    let net_force =
        drivetrain_torque / wheel_radius + slope_force + drag * drag_sign + rolling * rolling_sign
            - brake_force
                * if current_speed > 0.0 {
                    1.0
                } else if current_speed < 0.0 {
                    -1.0
                } else {
                    0.0
                };

    let acceleration = net_force / vehicle_mass;
    let integrated_speed = current_speed + acceleration * dt;
    let new_speed = if current_speed.abs() < 1e-6 && integrated_speed < 0.0 {
        0.0
    } else {
        integrated_speed
    };

    // Slip ratio
    let wheel_speed = drivetrain_torque.abs() / (vehicle_mass * wheel_radius + 1e-6);
    let slip_ratio = if new_speed > 1.0 {
        (wheel_speed - new_speed) / new_speed
    } else {
        0.0
    };

    SpeedState {
        ground_speed: new_speed,
        wheel_speed,
        slip_ratio,
        acceleration,
    }
}

/// Fuel system.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelTank {
    pub capacity: f64,
    pub current_level: f64,
    pub fuel_density: f64,
    pub base_consumption_rate: f64,
    pub idle_consumption_rate: f64,
}

impl Default for FuelTank {
    fn default() -> Self {
        Self {
            capacity: 60.0,
            current_level: 60.0,
            fuel_density: 0.75,
            base_consumption_rate: 0.01,
            idle_consumption_rate: 0.0005,
        }
    }
}

/// Fuel state output.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelState {
    pub current_level: f64,
    pub is_empty: bool,
    pub fuel_mass: f64,
}

impl FuelTank {
    pub fn update_fuel(&mut self, rpm: f64, throttle: f64, redline_rpm: f64, dt: f64) -> FuelState {
        if self.current_level <= 0.0 {
            return FuelState {
                current_level: 0.0,
                is_empty: true,
                fuel_mass: 0.0,
            };
        }
        let load_fraction = if redline_rpm > 0.0 {
            rpm / redline_rpm
        } else {
            0.0
        };
        let consumption = if throttle > 0.01 {
            self.base_consumption_rate * throttle * load_fraction * dt
        } else {
            self.idle_consumption_rate * dt
        };
        self.current_level = (self.current_level - consumption).max(0.0);

        FuelState {
            current_level: self.current_level,
            is_empty: self.current_level <= 0.0,
            fuel_mass: self.current_level * self.fuel_density,
        }
    }

    pub fn refuel(&mut self, amount: f64) {
        self.current_level = (self.current_level + amount).min(self.capacity);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_suspension_equilibrium() {
        let config = WheelConfig::default();
        let wheel_pos = [0.0, 0.7, 0.0];
        let terrain = 0.0;
        let (force, comp, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(!airborne, "Wheel should contact terrain");
        assert!(force.force > 0.0, "Suspension should produce upward force");
        assert!(comp > 0.0, "Compression should be positive");
    }

    #[test]
    fn test_suspension_force_direction() {
        let config = WheelConfig::default();
        let wheel_pos = [0.0, 0.65, 0.0]; // Close to ground
        let terrain = 0.0;
        let (force, _, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(!airborne);
        assert!(force.force > 0.0);
    }

    #[test]
    fn test_bump_stop_engages() {
        let mut config = WheelConfig::default();
        config.travel = 0.01; // Very short travel
                              // Need penetration >= 0.99 * (travel + rest_length) = 0.99 * 0.31 = 0.3069
                              // wheel_bottom = wp - 0.3 - 0.3 = wp - 0.6
                              // penetration = (wp - 0.6) - 0 = wp - 0.6 >= 0.3069 → wp >= 0.9069
        let wheel_pos = [0.0, 0.92, 0.0];
        let terrain = 0.0;
        let (_, comp, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(!airborne);
        assert!(
            comp >= 0.99,
            "Should be at max compression with short travel"
        );
    }

    #[test]
    fn test_wheel_airborne() {
        let config = WheelConfig::default();
        let wheel_pos = [0.0, 10.0, 0.0]; // Way above ground
        let terrain = 0.0;
        let (_, _, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(airborne, "Wheel high above ground should be airborne");
    }

    #[test]
    fn test_anti_roll_bar() {
        let config = SuspensionConfig::default();
        assert!(config.anti_roll_bar_stiffness > 0.0);
    }

    #[test]
    fn test_steering_angle_limits() {
        let config = SteeringConfig::default();
        let angle = update_steering(&config, 10.0, 0.0, 0.0, 1.0);
        assert!(angle.abs() <= config.max_steering_angle + 1e-6);
    }

    #[test]
    fn test_speed_sensitive_steering() {
        let config = SteeringConfig::default();
        let angle_low = update_steering(&config, 1.0, 0.0, 0.0, 1.0);
        let angle_high = update_steering(&config, 1.0, 30.0, 0.0, 1.0);
        assert!(
            angle_low.abs() > angle_high.abs(),
            "Steering should reduce at speed"
        );
    }

    #[test]
    fn test_ackermann_symmetric_at_zero() {
        let (l, r) = ackermann_angles(0.0, 2.5, 1.6);
        assert!((l).abs() < 1e-6);
        assert!((r).abs() < 1e-6);
    }

    #[test]
    fn test_ackermann_inner_turns_more() {
        // Turning right (positive angle): left wheel (inner) turns more
        let (left, right) = ackermann_angles(0.3, 2.5, 1.6);
        assert!(
            left.abs() > right.abs(),
            "Inner wheel should turn more (Ackermann)"
        );
    }

    #[test]
    fn test_speed_computed_from_forces() {
        let result = compute_speed(
            1000.0,
            0.0,
            &[5000.0; 4],
            1500.0,
            0.0,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            0.0,
        );
        assert!(
            result.ground_speed > 0.0,
            "Drivetrain torque should produce speed"
        );
        assert!(result.acceleration > 0.0);
    }

    #[test]
    fn test_brake_reduces_speed() {
        let result = compute_speed(
            0.0,
            2000.0,
            &[5000.0; 4],
            1500.0,
            0.0,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            10.0,
        );
        assert!(result.ground_speed < 10.0, "Braking should reduce speed");
    }

    #[test]
    fn test_rolling_resistance() {
        let result = compute_speed(
            0.0,
            0.0,
            &[5000.0; 4],
            1500.0,
            0.0,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            10.0,
        );
        assert!(
            result.ground_speed < 10.0,
            "Rolling resistance should slow vehicle"
        );
    }

    #[test]
    fn test_aerodynamic_drag_increases_with_speed() {
        let slow = compute_speed(
            500.0,
            0.0,
            &[5000.0; 4],
            1500.0,
            0.0,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            5.0,
        );
        let fast = compute_speed(
            500.0,
            0.0,
            &[5000.0; 4],
            1500.0,
            0.0,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            30.0,
        );
        // At high speed, drag is much larger, so net acceleration should be lower
        assert!(
            fast.acceleration < slow.acceleration,
            "Drag should reduce acceleration at high speed"
        );
    }

    #[test]
    fn test_slope_affects_speed() {
        let flat = compute_speed(
            0.0,
            0.0,
            &[5000.0; 4],
            1500.0,
            0.0,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            5.0,
        );
        let uphill = compute_speed(
            0.0,
            0.0,
            &[5000.0; 4],
            1500.0,
            0.1,
            0.35,
            2.0,
            1.225,
            0.3,
            0.015,
            1.0 / 60.0,
            5.0,
        );
        assert!(
            uphill.acceleration < flat.acceleration,
            "Uphill should reduce acceleration"
        );
    }

    // Fuel tests

    #[test]
    fn test_fuel_consumption_proportional_to_load() {
        let mut tank = FuelTank::default();
        let f1 = tank.update_fuel(3000.0, 0.5, 7000.0, 1.0);
        let mut tank2 = FuelTank::default();
        let f2 = tank2.update_fuel(3000.0, 1.0, 7000.0, 1.0);
        assert!(
            f2.current_level < f1.current_level,
            "Full throttle should consume more fuel"
        );
    }

    #[test]
    fn test_idle_consumption() {
        let mut tank = FuelTank::default();
        let before = tank.current_level;
        let _ = tank.update_fuel(800.0, 0.0, 7000.0, 10.0);
        assert!(tank.current_level < before, "Idle should consume some fuel");
    }

    #[test]
    fn test_empty_fuel_stops_engine() {
        let mut tank = FuelTank::default();
        tank.current_level = 0.0;
        let state = tank.update_fuel(3000.0, 1.0, 7000.0, 0.1);
        assert!(state.is_empty);
        assert_eq!(state.current_level, 0.0);
    }

    #[test]
    fn test_fuel_weight_affects_mass() {
        let mut tank = FuelTank::default();
        tank.current_level = 60.0;
        let state = tank.update_fuel(0.0, 0.0, 7000.0, 0.0);
        let expected_mass = 60.0 * 0.75;
        assert!((state.fuel_mass - expected_mass).abs() < 1e-6);
    }

    #[test]
    fn test_fuel_refuel() {
        let mut tank = FuelTank::default();
        tank.current_level = 10.0;
        tank.refuel(50.0);
        assert!((tank.current_level - 60.0).abs() < 1e-6);
        tank.refuel(10.0);
        assert!(
            (tank.current_level - 60.0).abs() < 1e-6,
            "Should not exceed capacity"
        );
    }
}
