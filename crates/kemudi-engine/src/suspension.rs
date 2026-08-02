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
    /// Previous terrain-to-mount suspension length used to derive the
    /// damper velocity from actual geometry rather than chassis Y velocity.
    pub previous_suspension_length: Option<f64>,
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
            previous_suspension_length: None,
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

/// Calculate the progressive force contributed by the final part of bump-stop
/// travel. The existing suspension schema exposes a single bump-stop rate, so
/// use a bounded quadratic engagement rather than inventing new configuration.
pub(crate) fn progressive_bump_stop_force(
    compression: f64,
    travel: f64,
    bump_stop_rate: f64,
) -> f64 {
    let safe_compression = if compression.is_finite() {
        compression.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let safe_travel = if travel.is_finite() {
        travel.max(0.0)
    } else {
        0.0
    };
    let safe_rate = if bump_stop_rate.is_finite() {
        bump_stop_rate.max(0.0)
    } else {
        0.0
    };
    let engagement = ((safe_compression - 0.88) / 0.12).clamp(0.0, 1.0);
    safe_rate * safe_travel * engagement * engagement
}

/// Raycast a wheel against terrain and compute suspension force.
pub fn raycast_wheel(
    config: &WheelConfig,
    wheel_pos: [f64; 3],
    suspension_length_velocity: f64,
    terrain_height: f64,
) -> (SuspensionForce, f64, bool) {
    raycast_wheel_with_normal(
        config,
        wheel_pos,
        suspension_length_velocity,
        terrain_height,
        [0.0, 1.0, 0.0],
    )
}

/// Raycast a wheel against a height-field surface while retaining its normal.
pub fn raycast_wheel_with_normal(
    config: &WheelConfig,
    wheel_pos: [f64; 3],
    suspension_length_velocity: f64,
    terrain_height: f64,
    surface_normal: [f64; 3],
) -> (SuspensionForce, f64, bool) {
    let normal_length = (surface_normal[0] * surface_normal[0]
        + surface_normal[1] * surface_normal[1]
        + surface_normal[2] * surface_normal[2])
        .sqrt();
    let normal = if normal_length > 1e-8 && normal_length.is_finite() {
        [
            surface_normal[0] / normal_length,
            surface_normal[1] / normal_length,
            surface_normal[2] / normal_length,
        ]
    } else {
        [0.0, 1.0, 0.0]
    };
    // The physics node is the suspension mount. The wheel center is below it
    // by the current suspension length, and the tire bottom must remain above
    // the terrain. A mount farther than rest_length + travel from the ground
    // has no contact; a lower mount compresses the suspension.
    let suspension_length = wheel_pos[1] - terrain_height - config.tire_radius;
    let max_extension = config.rest_length + config.travel;
    if suspension_length > max_extension + 0.01 {
        return (SuspensionForce::default(), 0.0, true);
    }

    // Contact exists. Keep the normalized compression at zero when the mount
    // is at or above its rest height and clamp hard impacts to full travel.
    let compression =
        ((config.rest_length - suspension_length) / config.travel.max(0.001)).clamp(0.0, 1.0);
    let contact_y = terrain_height;

    // Spring force
    let spring_force = config.spring_rate * compression * config.travel;

    // Damping force (opposes velocity)
    let compression_velocity = if suspension_length_velocity.is_finite() {
        suspension_length_velocity
    } else {
        0.0
    };
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
            normal,
        },
        compression,
        false,
    )
}

/// Rate of change between consecutive terrain-to-mount suspension lengths.
/// Sampling the complete length makes changing terrain height part of damper
/// motion even when the chassis has no world-space vertical velocity.
pub fn suspension_length_velocity(current_length: f64, previous_length: f64, dt: f64) -> f64 {
    if !current_length.is_finite() || !previous_length.is_finite() || !dt.is_finite() || dt <= 0.0 {
        return 0.0;
    }
    (current_length - previous_length) / dt
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
            speed_sensitivity: 0.04,
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
    let abs_angle = steering_angle.abs().min(std::f64::consts::FRAC_PI_2 - 1e-4);
    let safe_wheelbase = wheelbase.max(0.1);
    let half_track = track_width.max(0.1) * 0.5;
    // `steering_angle` is the virtual center-wheel angle. Ackermann geometry
    // derives both physical wheel angles from the common turn center; the old
    // cotangent expression accidentally made both wheels steer far beyond the
    // requested angle (0.5 rad became roughly 0.9-1.1 rad).
    let center_radius = safe_wheelbase / abs_angle.tan().max(1e-6);
    let inner = sign * (safe_wheelbase / (center_radius - half_track).max(0.05)).atan();
    let outer = sign * (safe_wheelbase / (center_radius + half_track)).atan();
    // Positive steering is a right turn in the input contract. The right
    // wheel is therefore the inside wheel for positive angles; the left wheel
    // is inside for negative angles.
    if steering_angle > 0.0 {
        (outer, inner)
    } else {
        (inner, outer)
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
        let load_fraction = if redline_rpm.is_finite() && redline_rpm > 0.0 {
            let ratio = rpm / redline_rpm;
            if ratio.is_finite() {
                ratio.clamp(0.0, 1.5)
            } else {
                0.0
            }
        } else {
            0.0
        };
        let safe_throttle = if throttle.is_finite() {
            throttle.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let safe_dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
        let consumption = if safe_throttle > 0.01 {
            self.base_consumption_rate.max(0.0) * safe_throttle * load_fraction * safe_dt
        } else {
            self.idle_consumption_rate.max(0.0) * safe_dt
        };
        self.current_level = (self.current_level - consumption).max(0.0);

        FuelState {
            current_level: self.current_level,
            is_empty: self.current_level <= 0.0,
            fuel_mass: self.current_level * self.fuel_density,
        }
    }

    pub fn refuel(&mut self, amount: f64) {
        if amount.is_finite() && amount > 0.0 {
            self.current_level = (self.current_level + amount).min(self.capacity.max(0.0));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_suspension_equilibrium() {
        let config = WheelConfig::default();
        let wheel_pos = [0.0, 0.55, 0.0];
        let terrain = 0.0;
        let (force, comp, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(!airborne, "Wheel should contact terrain");
        assert!(force.force > 0.0, "Suspension should produce upward force");
        assert!(comp > 0.0, "Compression should be positive");
    }

    #[test]
    fn test_suspension_force_direction() {
        let config = WheelConfig::default();
        let wheel_pos = [0.0, 0.5, 0.0]; // Compressed mount
        let terrain = 0.0;
        let (force, _, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(!airborne);
        assert!(force.force > 0.0);
    }

    #[test]
    fn test_suspension_preserves_surface_normal() {
        let config = WheelConfig::default();
        let normal = [0.0, 0.8660254, 0.5];
        let (force, _, airborne) =
            raycast_wheel_with_normal(&config, [0.0, 0.5, 0.0], 0.0, 0.0, normal);
        assert!(!airborne);
        assert!((force.normal[1] - normal[1]).abs() < 1e-6);
        assert!((force.normal[2] - normal[2]).abs() < 1e-6);
    }

    #[test]
    fn test_terrain_change_contributes_to_suspension_length_rate() {
        // The mount remains at the same world Y, but terrain rises 0.1 m
        // between samples, shortening the suspension by the same amount.
        let previous_length = 0.5 - 0.0 - 0.3;
        let current_length = 0.5 - 0.1 - 0.3;
        let length_velocity = suspension_length_velocity(current_length, previous_length, 0.1);
        assert!(
            (length_velocity + 1.0).abs() < 1e-9,
            "rising terrain should contribute to damper compression velocity"
        );
    }

    #[test]
    fn test_bump_stop_engages() {
        let mut config = WheelConfig::default();
        config.travel = 0.01; // Very short travel
        let wheel_pos = [0.0, 0.590, 0.0];
        let terrain = 0.0;
        let (_, comp, airborne) = raycast_wheel(&config, wheel_pos, 0.0, terrain);
        assert!(!airborne);
        assert!(
            comp >= 0.99,
            "Should be at max compression with short travel"
        );
    }

    #[test]
    fn test_bump_stop_force_is_progressive_near_bottom_out() {
        let early = progressive_bump_stop_force(0.92, 0.2, 500_000.0);
        let late = progressive_bump_stop_force(0.99, 0.2, 500_000.0);

        assert!(early > 0.0);
        assert!(
            late > early * 2.0,
            "late bump stop force must rise progressively"
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
        // Turning right (positive angle): right wheel (inner) turns more.
        let (left, right) = ackermann_angles(0.3, 2.5, 1.6);
        assert!(
            left < 0.3 && right > 0.3,
            "wheel angles must straddle the requested center angle"
        );
        assert!(
            right.abs() > left.abs(),
            "Inner wheel should turn more (Ackermann)"
        );
        let outer_radius = 2.5 / left.tan();
        let inner_radius = 2.5 / right.tan();
        assert!((outer_radius - inner_radius - 1.6).abs() < 1e-9);
    }

    #[test]
    fn test_ackermann_left_turn_uses_left_inner_wheel() {
        let (left, right) = ackermann_angles(-0.3, 2.5, 1.6);
        assert!(
            left.abs() > right.abs(),
            "The left wheel should be the inside wheel for a left turn"
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
