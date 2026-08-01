use serde::{Deserialize, Serialize};

/// Gear ratios for a transmission
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Transmission {
    pub mode: TransmissionMode,
    pub gear_ratios: Vec<f64>,
    pub final_drive: f64,
    pub current_gear: i32,
    pub reverse_ratio: f64,
    pub clutch_engagement: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TransmissionMode {
    Manual,
    Automatic,
}

impl Default for Transmission {
    fn default() -> Self {
        Self {
            mode: TransmissionMode::Manual,
            gear_ratios: vec![3.5, 2.1, 1.4, 1.0, 0.7],
            final_drive: 3.7,
            current_gear: 1,
            reverse_ratio: -3.2,
            clutch_engagement: 1.0,
        }
    }
}

impl Transmission {
    pub fn total_ratio(&self) -> f64 {
        if self.current_gear == 0 {
            return 0.0;
        }
        let gear = if self.current_gear < 0 {
            self.reverse_ratio
        } else if self.current_gear > 0 && (self.current_gear as usize) <= self.gear_ratios.len() {
            self.gear_ratios[(self.current_gear as usize) - 1]
        } else {
            return 0.0;
        };
        gear * self.final_drive
    }

    pub fn engine_rpm_from_wheel_speed(&self, wheel_speed: f64) -> f64 {
        let ratio = self.total_ratio();
        if ratio.abs() < 1e-6 {
            return 0.0;
        }
        wheel_speed.abs() * ratio.abs() * 60.0 / (2.0 * std::f64::consts::PI)
    }

    pub fn wheel_torque(&self, engine_torque: f64) -> f64 {
        let ratio = self.total_ratio();
        engine_torque * ratio * self.clutch_engagement
    }

    pub fn shift_up(&mut self) -> bool {
        let max_gear = self.gear_ratios.len() as i32;
        if self.current_gear < max_gear {
            self.current_gear += 1;
            true
        } else {
            false
        }
    }

    pub fn shift_down(&mut self) -> bool {
        if self.current_gear > -1 {
            self.current_gear -= 1;
            true
        } else {
            false
        }
    }
}

// === Shift timing ===

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ShiftPhase {
    Idle,
    Disengaging,
    Neutral,
    Engaging,
}

/// Cause of engine over-rev, distinguishing rev-limiter from drivetrain-forced.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum OverrevCause {
    /// Normal throttle-driven revving (limited by rev limiter).
    ThrottleDriven,
    /// Rapid downshift that forces RPM above safe limit.
    RapidDownshift,
    /// Clutch dump with speed mismatch.
    ClutchDump,
    /// Wheel hop / drivetrain oscillation.
    WheelHop,
    /// Downhill engine braking in low gear.
    EngineBraking,
}

/// Transient shock from clutch engagement speed mismatch.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ClutchShock {
    /// Current shock magnitude (0 = no shock, 1 = maximum).
    pub magnitude: f64,
    /// Shock decay rate per second.
    pub decay_rate: f64,
    /// Driveline torque spike from shock (Nm).
    pub torque_spike: f64,
}

impl Default for ClutchShock {
    fn default() -> Self {
        Self {
            magnitude: 0.0,
            decay_rate: 8.0,
            torque_spike: 0.0,
        }
    }
}

impl ClutchShock {
    pub fn apply(&mut self, rpm_mismatch: f64, clutch_engagement: f64, dt: f64) {
        // Shock proportional to RPM mismatch and engagement speed
        let raw_shock = (rpm_mismatch.abs() / 1000.0).min(1.0) * clutch_engagement;
        let decay = (1.0 - self.decay_rate.max(0.0) * dt.max(0.0)).max(0.0);
        self.magnitude = raw_shock.max(self.magnitude * decay);
        self.torque_spike = raw_shock * 200.0; // Nm spike
    }
}

/// Drivetrain configuration validation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrivetrainConfig {
    pub gear_ratios: Vec<f64>,
    pub final_drive: f64,
    pub reverse_ratio: f64,
    pub max_gears: usize,
}

impl DrivetrainConfig {
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.gear_ratios.is_empty() {
            errors.push("gear_ratios must not be empty".into());
        }
        if self.final_drive <= 0.0 {
            errors.push("final_drive must be positive".into());
        }
        if self.reverse_ratio >= 0.0 {
            errors.push("reverse_ratio must be negative".into());
        }
        // Forward gears should be descending (higher ratio in lower gears)
        for w in self.gear_ratios.windows(2) {
            if w[0] < w[1] {
                errors.push(format!(
                    "gear_ratios must be descending: {} < {}",
                    w[0], w[1]
                ));
            }
        }
        for (i, r) in self.gear_ratios.iter().enumerate() {
            if *r <= 0.0 {
                errors.push(format!("gear_ratio[{}] must be positive, got {}", i, r));
            }
        }
        if self.gear_ratios.len() > self.max_gears {
            errors.push(format!(
                "too many gears: {} > max {}",
                self.gear_ratios.len(),
                self.max_gears
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

// === Automatic shifting ===

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutoShiftLogic {
    pub upshift_rpm: f64,
    pub downshift_rpm: f64,
    pub shift_delay: f64,
    pub kickdown_threshold: f64,
    pub(crate) last_shift_time: f64,
}

impl Default for AutoShiftLogic {
    fn default() -> Self {
        Self {
            upshift_rpm: 6000.0,
            downshift_rpm: 2500.0,
            shift_delay: 0.5,
            kickdown_threshold: 0.8,
            last_shift_time: 0.0,
        }
    }
}

impl AutoShiftLogic {
    pub fn should_upshift(&self, current_gear: i32, rpm: f64, throttle: f64, time: f64) -> bool {
        if current_gear <= 0 || time - self.last_shift_time < self.shift_delay {
            return false;
        }
        let effective_upshift = if throttle > self.kickdown_threshold {
            self.upshift_rpm - 500.0 // kickdown: shift earlier
        } else {
            self.upshift_rpm
        };
        rpm >= effective_upshift
    }

    pub fn should_downshift(&self, current_gear: i32, rpm: f64, time: f64) -> bool {
        if current_gear <= 1 || time - self.last_shift_time < self.shift_delay {
            return false;
        }
        rpm <= self.downshift_rpm
    }

    pub fn record_shift(&mut self, time: f64) {
        self.last_shift_time = time;
    }
}
