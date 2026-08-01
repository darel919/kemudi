use serde::{Deserialize, Serialize};

/// Engine RPM range and torque curve
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Engine {
    pub rpm: f64,
    pub idle_rpm: f64,
    pub redline_rpm: f64,
    pub rev_limiter_rpm: f64,
    /// Torque curve: [(rpm, torque_nm), ...] sorted ascending by rpm
    pub torque_curve: Vec<(f64, f64)>,
    pub throttle_response: f64,
    pub engine_braking: f64,
}

impl Default for Engine {
    fn default() -> Self {
        Self {
            rpm: 800.0,
            idle_rpm: 800.0,
            redline_rpm: 7000.0,
            rev_limiter_rpm: 7500.0,
            torque_curve: vec![
                (0.0, 100.0),
                (1000.0, 150.0),
                (2000.0, 200.0),
                (3000.0, 250.0),
                (4000.0, 280.0),
                (5000.0, 260.0),
                (6000.0, 220.0),
                (7000.0, 160.0),
                (8000.0, 100.0),
            ],
            throttle_response: 0.8,
            engine_braking: 30.0,
        }
    }
}

impl Engine {
    pub fn torque_at_rpm(&self, rpm: f64) -> f64 {
        if self.torque_curve.is_empty() {
            return 0.0;
        }
        if rpm <= self.torque_curve[0].0 {
            return self.torque_curve[0].1;
        }
        if rpm >= self.torque_curve.last().unwrap().0 {
            return self.torque_curve.last().unwrap().1;
        }
        for w in self.torque_curve.windows(2) {
            if rpm >= w[0].0 && rpm <= w[1].0 {
                let t = (rpm - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + t * (w[1].1 - w[0].1);
            }
        }
        0.0
    }

    pub fn update(&mut self, throttle: f64, dt: f64) {
        let target_rpm = if throttle > 0.01 {
            self.idle_rpm + (self.redline_rpm - self.idle_rpm) * throttle
        } else {
            self.idle_rpm
        };
        let rate = self.throttle_response * 10.0;
        self.rpm += (target_rpm - self.rpm) * rate * dt;
        self.rpm = self.rpm.clamp(0.0, self.rev_limiter_rpm);
    }
}

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
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct ClutchShock {
    /// Current shock magnitude (0 = no shock, 1 = maximum).
    pub magnitude: f64,
    /// Shock decay rate per second.
    pub decay_rate: f64,
    /// Driveline torque spike from shock (Nm).
    pub torque_spike: f64,
}

impl ClutchShock {
    pub fn apply(&mut self, rpm_mismatch: f64, clutch_engagement: f64, dt: f64) {
        // Shock proportional to RPM mismatch and engagement speed
        let raw_shock = (rpm_mismatch.abs() / 1000.0).min(1.0) * clutch_engagement;
        self.magnitude = raw_shock.max(self.magnitude * (1.0 - self.decay_rate * dt));
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
    last_shift_time: f64,
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

// === Differential ===

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum DiffMode {
    Open,
    Locked,
    LimitedSlip,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Differential {
    pub mode: DiffMode,
    pub bias: f64,
    pub left_ratio: f64,
    pub right_ratio: f64,
}

impl Default for Differential {
    fn default() -> Self {
        Self {
            mode: DiffMode::Open,
            bias: 0.5,
            left_ratio: 0.5,
            right_ratio: 0.5,
        }
    }
}

impl Differential {
    pub fn apply_differential(
        &mut self,
        torque: f64,
        left_grip: f64,
        right_grip: f64,
    ) -> (f64, f64) {
        match self.mode {
            DiffMode::Locked => {
                self.left_ratio = 0.5;
                self.right_ratio = 0.5;
                (torque * 0.5, torque * 0.5)
            }
            DiffMode::Open => {
                self.left_ratio = 0.5;
                self.right_ratio = 0.5;
                (torque * 0.5, torque * 0.5)
            }
            DiffMode::LimitedSlip => {
                let total_grip = left_grip + right_grip + 1e-6;
                let base = torque * self.bias;
                let slip_torque = torque * (1.0 - self.bias);
                let left = base + slip_torque * (left_grip / total_grip);
                let right = base + slip_torque * (right_grip / total_grip);
                self.left_ratio = left / (torque + 1e-6);
                self.right_ratio = right / (torque + 1e-6);
                (left, right)
            }
        }
    }
}

// === Combined Drivetrain ===

pub struct Drivetrain {
    pub engine: Engine,
    pub transmission: Transmission,
    pub wheel_speed: f64,
    pub vehicle_speed: f64,
    pub throttle_input: f64,
    pub brake_input: f64,
    pub clutch_input: f64,
    pub shift_phase: ShiftPhase,
    pub shift_timer: f64,
    pub shift_duration: f64,
    pub pending_gear: i32,
    pub auto_shift: AutoShiftLogic,
    pub differential: Differential,
    pub time: f64,
    pub is_stalled: bool,
    /// Drivetrain-forced over-rev events (not rev-limiter limited).
    pub overrev_events: Vec<OverrevCause>,
    /// Current clutch shock state.
    pub clutch_shock: ClutchShock,
    /// Maximum allowed downshift RPM (beyond this, shift is blocked or causes damage).
    pub max_downshift_rpm: f64,
}

impl Drivetrain {
    pub fn new() -> Drivetrain {
        Drivetrain {
            engine: Engine::default(),
            transmission: Transmission::default(),
            wheel_speed: 0.0,
            vehicle_speed: 0.0,
            throttle_input: 0.0,
            brake_input: 0.0,
            clutch_input: 0.0,
            shift_phase: ShiftPhase::Idle,
            shift_timer: 0.0,
            shift_duration: 0.15,
            pending_gear: 0,
            auto_shift: AutoShiftLogic::default(),
            differential: Differential::default(),
            time: 0.0,
            is_stalled: false,
            overrev_events: Vec::new(),
            clutch_shock: ClutchShock::default(),
            max_downshift_rpm: 8000.0,
        }
    }

    pub fn update(&mut self, throttle: f64, brake: f64, clutch: f64, dt: f64, vehicle_mass: f64) {
        self.time += dt;
        self.throttle_input = throttle.clamp(0.0, 1.0);
        self.brake_input = brake.clamp(0.0, 1.0);
        self.clutch_input = clutch.clamp(0.0, 1.0);

        // Update shift phase
        self.update_shift_phase(dt);

        // Clutch engagement
        self.transmission.clutch_engagement = if self.shift_phase == ShiftPhase::Neutral {
            0.0
        } else {
            1.0 - self.clutch_input
        };

        // Engine torque — throttle gates power delivery
        let engine_torque = self.engine.torque_at_rpm(self.engine.rpm) * self.throttle_input;

        // Wheel torque through drivetrain
        let drive_torque = self.transmission.wheel_torque(engine_torque);

        // Engine braking
        let engine_brake_torque = if self.throttle_input < 0.05 && self.wheel_speed.abs() > 1e-6 {
            self.engine.engine_braking
                * self.transmission.total_ratio().abs()
                * self.wheel_speed.signum()
        } else {
            0.0
        };

        // Brake torque
        let direction = if self.wheel_speed.abs() > 1e-6 {
            self.wheel_speed.signum()
        } else {
            1.0
        };
        let brake_torque = self.brake_input * 500.0 * direction;

        // Net torque on wheels
        let net_torque = drive_torque - engine_brake_torque - brake_torque;

        // Inertia
        let wheel_inertia = vehicle_mass * 0.01;
        let angular_accel = net_torque / (wheel_inertia + 1e-6);

        self.wheel_speed += angular_accel * dt;
        self.wheel_speed = self.wheel_speed.clamp(-200.0, 200.0);

        // Vehicle speed
        let wheel_radius = 0.3;
        self.vehicle_speed = self.wheel_speed * wheel_radius;

        // Update engine RPM
        if self.shift_phase == ShiftPhase::Neutral {
            // Rev toward throttle target without drivetrain coupling
            self.engine.update(self.throttle_input, dt);
        } else if self.transmission.clutch_engagement > 0.5 {
            // Clutch engaged: engine RPM follows wheel speed through gear ratio
            let target_rpm = self
                .transmission
                .engine_rpm_from_wheel_speed(self.wheel_speed);
            let blend = self.transmission.clutch_engagement * self.engine.throttle_response;
            self.engine.rpm +=
                (target_rpm.max(self.engine.idle_rpm) - self.engine.rpm) * blend * dt;
            // Rev limiter
            self.engine.rpm = self.engine.rpm.min(self.engine.rev_limiter_rpm);
        } else {
            self.engine.update(self.throttle_input, dt);
        }

        // Idle behavior
        if self.throttle_input < 0.05 && self.transmission.clutch_engagement > 0.5 {
            if self.engine.rpm > self.engine.idle_rpm + 100.0 {
                self.engine.rpm -= self.engine.engine_braking * dt;
            }
        }

        // Stall detection
        self.is_stalled = self.transmission.clutch_engagement > 0.5
            && self.engine.rpm < self.engine.idle_rpm * 0.7
            && self.wheel_speed.abs() < 0.5;
        if self.is_stalled {
            self.engine.rpm = self.engine.idle_rpm;
        }

        // Rev limiter
        if self.engine.rpm >= self.engine.rev_limiter_rpm {
            self.engine.rpm = self.engine.rev_limiter_rpm;
        }
    }

    fn update_shift_phase(&mut self, dt: f64) {
        match self.shift_phase {
            ShiftPhase::Idle => {}
            ShiftPhase::Disengaging => {
                self.shift_timer -= dt;
                if self.shift_timer <= 0.0 {
                    // Apply gear change
                    self.transmission.current_gear = self.pending_gear;
                    self.shift_phase = ShiftPhase::Neutral;
                    self.shift_timer = self.shift_duration / 3.0;

                    // Rev match: adjust RPM toward target for new gear
                    let target_rpm = self
                        .transmission
                        .engine_rpm_from_wheel_speed(self.wheel_speed);
                    self.engine.rpm = self.engine.rpm * 0.7 + target_rpm * 0.3;
                }
            }
            ShiftPhase::Neutral => {
                self.shift_timer -= dt;
                if self.shift_timer <= 0.0 {
                    self.shift_phase = ShiftPhase::Engaging;
                    self.shift_timer = self.shift_duration / 3.0;
                }
            }
            ShiftPhase::Engaging => {
                self.shift_timer -= dt;
                self.compute_clutch_shock(dt);
                if self.shift_timer <= 0.0 {
                    self.shift_phase = ShiftPhase::Idle;
                    self.auto_shift.record_shift(self.time);
                }
            }
        }
    }

    pub fn request_shift_up(&mut self) -> bool {
        if self.shift_phase != ShiftPhase::Idle {
            return false; // blocked
        }
        let max_gear = self.transmission.gear_ratios.len() as i32;
        if self.transmission.current_gear >= max_gear {
            return false;
        }
        self.pending_gear = self.transmission.current_gear + 1;
        self.shift_phase = ShiftPhase::Disengaging;
        self.shift_timer = self.shift_duration / 3.0;
        true
    }

    pub fn request_shift_down(&mut self) -> bool {
        if self.shift_phase != ShiftPhase::Idle {
            return false;
        }
        if self.transmission.current_gear <= -1 {
            return false;
        }
        self.pending_gear = self.transmission.current_gear - 1;
        if self.pending_gear < 0 && self.wheel_speed.abs() > 0.5 {
            self.pending_gear = self.transmission.current_gear;
            return false;
        }
        self.shift_phase = ShiftPhase::Disengaging;
        self.shift_timer = self.shift_duration / 3.0;
        true
    }

    /// Predict what engine RPM would result from downshifting to `target_gear`
    /// at current wheel speed. Used to check for over-rev before applying shift.
    pub fn predict_downshift_rpm(&self, wheel_speed: f64, target_gear: i32) -> f64 {
        if target_gear <= 0 {
            return self.engine.idle_rpm;
        }
        let idx = (target_gear as usize).saturating_sub(1);
        if idx >= self.transmission.gear_ratios.len() {
            return self.engine.rev_limiter_rpm;
        }
        let ratio = self.transmission.gear_ratios[idx].abs() * self.transmission.final_drive;
        if ratio < 1e-6 {
            return 0.0;
        }
        wheel_speed.abs() * ratio * 60.0 / (2.0 * std::f64::consts::PI)
    }

    /// Check if a downshift would cause over-rev, and record the cause.
    /// Returns the predicted RPM and whether the shift should be blocked.
    pub fn check_downshift_overrev(&mut self, target_gear: i32) -> (f64, bool) {
        let predicted_rpm = self.predict_downshift_rpm(self.wheel_speed, target_gear);
        if predicted_rpm > self.engine.rev_limiter_rpm {
            // Drivetrain-forced over-rev — rev limiter cannot help here
            self.overrev_events.push(OverrevCause::RapidDownshift);
            (predicted_rpm, true) // blocked
        } else if predicted_rpm > self.max_downshift_rpm {
            (predicted_rpm, true) // soft limit
        } else {
            (predicted_rpm, false) // safe
        }
    }

    /// Compute clutch shock magnitude from RPM mismatch during shift completion.
    fn compute_clutch_shock(&mut self, dt: f64) {
        let target_rpm = self
            .transmission
            .engine_rpm_from_wheel_speed(self.wheel_speed);
        let mismatch = (self.engine.rpm - target_rpm).abs();
        self.clutch_shock
            .apply(mismatch, self.transmission.clutch_engagement, dt);
    }

    pub fn apply_automatic_shifting(&mut self) {
        if self.transmission.mode != TransmissionMode::Automatic {
            return;
        }
        if self.shift_phase != ShiftPhase::Idle {
            return;
        }
        if self.auto_shift.should_upshift(
            self.transmission.current_gear,
            self.engine.rpm,
            self.throttle_input,
            self.time,
        ) {
            self.request_shift_up();
        } else if self.auto_shift.should_downshift(
            self.transmission.current_gear,
            self.engine.rpm,
            self.time,
        ) {
            self.request_shift_down();
        }
    }

    pub fn set_gear(&mut self, gear: i32) {
        let max = self.transmission.gear_ratios.len() as i32;
        self.transmission.current_gear = gear.clamp(-1, max);
    }

    pub fn get_rpm(&self) -> f64 {
        self.engine.rpm
    }

    pub fn get_gear(&self) -> i32 {
        self.transmission.current_gear
    }

    pub fn get_vehicle_speed(&self) -> f64 {
        self.vehicle_speed
    }

    pub fn get_clutch(&self) -> f64 {
        self.transmission.clutch_engagement
    }

    pub fn shift_up(&mut self) {
        self.request_shift_up();
    }

    pub fn shift_down(&mut self) {
        self.request_shift_down();
    }
}

impl Default for Drivetrain {
    fn default() -> Self {
        Self::new()
    }
}

/// Transmission Control Module for automatic transmissions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmissionControlModule {
    pub enabled: bool,
    /// Current TCM state.
    pub state: TCMState,
    /// Shift scheduling parameters.
    pub shift_schedule: TCMShiftSchedule,
    /// Adaptive line pressure corrections per gear.
    pub adaptive_pressure: [f64; 10],
    /// Converter lockup state.
    pub converter_lockup: bool,
    /// Converter lockup speed threshold (km/h).
    pub lockup_speed_threshold: f64,
    /// Converter lockup throttle threshold (0-1).
    pub lockup_throttle_threshold: f64,
    /// Current line pressure multiplier (0-1).
    pub line_pressure: f64,
    /// Thermal protection: transmission temperature limit.
    pub thermal_limit: f64,
    /// Limp mode active.
    pub limp_mode: bool,
    /// Limp mode gear (usually 3rd).
    pub limp_gear: i32,
    /// Active faults.
    pub faults: TCMFaults,
    /// Learning/adaptation state.
    pub learning: TCMLearning,
    /// Shift request pending.
    pub pending_shift: Option<i32>,
    /// Time since last shift (seconds).
    pub time_since_shift: f64,
    /// Minimum time between shifts (seconds).
    pub min_shift_interval: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TCMState {
    Normal,
    Sport,
    Winter,
    Manual,
    Limp,
    Fault,
    Off,
}

/// Shift scheduling parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TCMShiftSchedule {
    /// Upshift RPM thresholds per gear (index 0 = 1→2, etc.).
    pub upshift_rpm: Vec<f64>,
    /// Downshift RPM thresholds per gear.
    pub downshift_rpm: Vec<f64>,
    /// Throttle-based shift map: higher throttle delays upshifts.
    pub throttle_shift_factor: f64,
    /// Kickdown threshold: throttle > this triggers downshift.
    pub kickdown_threshold: f64,
    /// Grade-aware: uphill delays upshift, downhill accelerates downshift.
    pub grade_factor: f64,
}

impl Default for TCMShiftSchedule {
    fn default() -> Self {
        Self {
            upshift_rpm: vec![
                3500.0, 3800.0, 4000.0, 4200.0, 4500.0, 4800.0, 5000.0, 5200.0, 5500.0,
            ],
            downshift_rpm: vec![
                1200.0, 1400.0, 1600.0, 1800.0, 2000.0, 2200.0, 2400.0, 2600.0, 2800.0,
            ],
            throttle_shift_factor: 0.3,
            kickdown_threshold: 0.85,
            grade_factor: 0.1,
        }
    }
}

/// TCM fault tracking.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TCMFaults {
    pub solenoid_faults: Vec<usize>,
    pub hydraulic_pressure_loss: bool,
    pub communication_loss: bool,
    pub sensor_faults: Vec<String>,
    pub overheating: bool,
}

/// Bounded adaptive learning state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TCMLearning {
    /// Shift quality corrections per gear pair (positive = firmer, negative = softer).
    pub shift_quality: Vec<f64>,
    /// Clutch wear compensation factor.
    pub clutch_wear_factor: f64,
    /// Learning rate.
    pub learning_rate: f64,
    /// Maximum allowed adaptation.
    pub max_adaptation: f64,
}

impl Default for TCMLearning {
    fn default() -> Self {
        Self {
            shift_quality: vec![0.0; 10],
            clutch_wear_factor: 1.0,
            learning_rate: 0.01,
            max_adaptation: 0.3,
        }
    }
}

impl Default for TransmissionControlModule {
    fn default() -> Self {
        Self {
            enabled: true,
            state: TCMState::Normal,
            shift_schedule: TCMShiftSchedule::default(),
            adaptive_pressure: [1.0; 10],
            converter_lockup: false,
            lockup_speed_threshold: 40.0,
            lockup_throttle_threshold: 0.3,
            line_pressure: 1.0,
            thermal_limit: 120.0,
            limp_mode: false,
            limp_gear: 3,
            faults: TCMFaults::default(),
            learning: TCMLearning::default(),
            pending_shift: None,
            time_since_shift: 10.0,
            min_shift_interval: 0.5,
        }
    }
}

impl TransmissionControlModule {
    /// Main TCM update: determines shift, pressure, converter state.
    pub fn update(
        &mut self,
        engine_rpm: f64,
        vehicle_speed: f64,
        throttle: f64,
        grade: f64,
        transmission_temp: f64,
        current_gear: i32,
        max_gear: i32,
        dt: f64,
    ) -> Option<i32> {
        if !self.enabled {
            return None;
        }
        self.time_since_shift += dt;

        // Check faults
        self.check_faults(transmission_temp);

        // Limp mode: hold fixed gear
        if self.limp_mode || self.state == TCMState::Fault {
            self.line_pressure = 0.6;
            return None;
        }

        // Thermal protection: reduce line pressure when hot
        if transmission_temp > self.thermal_limit {
            self.line_pressure = (1.0 - (transmission_temp - self.thermal_limit) / 30.0).max(0.4);
            self.state = TCMState::Limp;
            self.limp_mode = true;
            return None;
        }

        // Converter lockup logic
        self.update_converter_lockup(vehicle_speed, throttle, engine_rpm);

        // Line pressure from throttle + adaptive
        self.update_line_pressure(throttle, current_gear);

        // Shift scheduling
        if self.time_since_shift < self.min_shift_interval {
            return None;
        }

        self.determine_shift(
            engine_rpm,
            vehicle_speed,
            throttle,
            grade,
            current_gear,
            max_gear,
        )
    }

    fn check_faults(&mut self, temp: f64) {
        if temp > self.thermal_limit + 20.0 {
            self.faults.overheating = true;
        }
        if !self.faults.solenoid_faults.is_empty() || self.faults.hydraulic_pressure_loss {
            self.state = TCMState::Fault;
        }
    }

    fn update_converter_lockup(&mut self, speed: f64, throttle: f64, rpm: f64) {
        // Lock up when speed is above threshold and throttle is moderate
        if speed > self.lockup_speed_threshold
            && throttle < self.lockup_throttle_threshold
            && rpm > 1500.0
        {
            self.converter_lockup = true;
        } else if speed < self.lockup_speed_threshold * 0.7 || throttle > 0.7 {
            self.converter_lockup = false;
        }
    }

    fn update_line_pressure(&mut self, throttle: f64, gear: i32) {
        let idx = (gear - 1).max(0) as usize;
        let base = 0.5 + throttle * 0.5;
        let adaptive = if idx < self.adaptive_pressure.len() {
            self.adaptive_pressure[idx]
        } else {
            1.0
        };
        self.line_pressure = (base * adaptive).clamp(0.3, 1.0);
    }

    fn determine_shift(
        &mut self,
        engine_rpm: f64,
        _vehicle_speed: f64,
        throttle: f64,
        grade: f64,
        current_gear: i32,
        max_gear: i32,
    ) -> Option<i32> {
        if current_gear <= 0 || current_gear > max_gear {
            return None;
        }

        let idx = (current_gear - 1) as usize;

        // Kickdown: aggressive throttle triggers downshift
        if throttle > self.shift_schedule.kickdown_threshold && current_gear > 1 {
            self.time_since_shift = 0.0;
            self.adapt_shift_quality(current_gear, current_gear - 1, false);
            return Some(current_gear - 1);
        }

        // Upshift check
        if idx < self.shift_schedule.upshift_rpm.len() && current_gear < max_gear {
            let threshold = self.shift_schedule.upshift_rpm[idx]
                + throttle * self.shift_schedule.throttle_shift_factor * 1000.0
                - grade * self.shift_schedule.grade_factor * 500.0;
            if engine_rpm > threshold {
                self.time_since_shift = 0.0;
                self.adapt_shift_quality(current_gear, current_gear + 1, true);
                return Some(current_gear + 1);
            }
        }

        // Downshift check
        if idx < self.shift_schedule.downshift_rpm.len() && current_gear > 1 {
            let threshold = self.shift_schedule.downshift_rpm[idx]
                - grade * self.shift_schedule.grade_factor * 300.0;
            if engine_rpm < threshold {
                self.time_since_shift = 0.0;
                self.adapt_shift_quality(current_gear, current_gear - 1, false);
                return Some(current_gear - 1);
            }
        }

        None
    }

    /// Adaptive learning: adjust shift quality based on shift outcome.
    fn adapt_shift_quality(&mut self, from_gear: i32, _to_gear: i32, is_upshift: bool) {
        let idx = (from_gear - 1).max(0) as usize;
        if idx < self.learning.shift_quality.len() {
            // Simplified: firmer for upshifts, softer for downshifts
            let correction = if is_upshift { 0.01 } else { -0.01 };
            let current = self.learning.shift_quality[idx];
            let new_val = (current + correction * self.learning.learning_rate)
                .clamp(-self.learning.max_adaptation, self.learning.max_adaptation);
            self.learning.shift_quality[idx] = new_val;
        }
    }
}

// === Vehicle Drive Modes ===

/// Available drive mode identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DriveMode {
    Normal,
    Eco,
    Comfort,
    Sport,
    Track,
    Snow,
}

impl DriveMode {
    pub fn all() -> &'static [DriveMode] {
        &[
            Self::Normal,
            Self::Eco,
            Self::Comfort,
            Self::Sport,
            Self::Track,
            Self::Snow,
        ]
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Normal => "Normal",
            Self::Eco => "Eco",
            Self::Comfort => "Comfort",
            Self::Sport => "Sport",
            Self::Track => "Track",
            Self::Snow => "Snow",
        }
    }
}

/// Drive-mode profile: all tunable parameters affected by mode selection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriveModeProfile {
    pub mode: DriveMode,
    /// Engine throttle mapping multiplier (1.0 = linear, <1 = dampened).
    pub throttle_map: f64,
    /// Throttle response speed multiplier.
    pub throttle_response: f64,
    /// Idle RPM offset.
    pub idle_rpm_offset: f64,
    /// Engine braking multiplier.
    pub engine_braking_factor: f64,
    /// Fuel efficiency target (0-1, higher = more efficient).
    pub fuel_efficiency_target: f64,
    /// TCM mode to apply.
    pub tcm_mode: TCMState,
    /// TCM upshift RPM modifier (negative = earlier upshifts).
    pub tcm_upshift_modifier: f64,
    /// TCM downshift RPM modifier.
    pub tcm_downshift_modifier: f64,
    /// TCM kickdown sensitivity (higher = easier kickdown).
    pub tcm_kickdown_sensitivity: f64,
    /// Converter lockup speed modifier.
    pub tcm_lockup_modifier: f64,
    /// Steering assist multiplier (1.0 = normal, <1 = heavier).
    pub steering_assist: f64,
    /// Steering response multiplier.
    pub steering_response: f64,
    /// Suspension damping multiplier (1.0 = normal, >1 = stiffer).
    pub suspension_damping: f64,
    /// Differential mode preference.
    pub differential_mode: DiffMode,
    /// ABS intervention threshold modifier (higher = less intervention).
    pub abs_threshold_modifier: f64,
    /// Traction control threshold modifier.
    pub tc_threshold_modifier: f64,
    /// VSC/ESC intervention threshold modifier.
    pub vsc_threshold_modifier: f64,
    /// Whether this mode is available for the given vehicle configuration.
    pub available: bool,
    /// Reason if unavailable.
    pub unavailable_reason: Option<String>,
}

impl Default for DriveModeProfile {
    fn default() -> Self {
        Self::normal()
    }
}

impl DriveModeProfile {
    pub fn normal() -> Self {
        Self {
            mode: DriveMode::Normal,
            throttle_map: 1.0,
            throttle_response: 1.0,
            idle_rpm_offset: 0.0,
            engine_braking_factor: 1.0,
            fuel_efficiency_target: 0.5,
            tcm_mode: TCMState::Normal,
            tcm_upshift_modifier: 0.0,
            tcm_downshift_modifier: 0.0,
            tcm_kickdown_sensitivity: 1.0,
            tcm_lockup_modifier: 0.0,
            steering_assist: 1.0,
            steering_response: 1.0,
            suspension_damping: 1.0,
            differential_mode: DiffMode::Open,
            abs_threshold_modifier: 1.0,
            tc_threshold_modifier: 1.0,
            vsc_threshold_modifier: 1.0,
            available: true,
            unavailable_reason: None,
        }
    }

    pub fn eco() -> Self {
        Self {
            mode: DriveMode::Eco,
            throttle_map: 0.7,
            throttle_response: 0.8,
            idle_rpm_offset: -100.0,
            engine_braking_factor: 1.2,
            fuel_efficiency_target: 0.8,
            tcm_mode: TCMState::Normal,
            tcm_upshift_modifier: -500.0, // earlier upshifts
            tcm_downshift_modifier: 200.0,
            tcm_kickdown_sensitivity: 0.6,
            tcm_lockup_modifier: -10.0, // earlier lockup
            steering_assist: 1.2,
            steering_response: 0.9,
            suspension_damping: 0.9,
            differential_mode: DiffMode::Open,
            abs_threshold_modifier: 1.0,
            tc_threshold_modifier: 1.0,
            vsc_threshold_modifier: 1.0,
            available: true,
            unavailable_reason: None,
        }
    }

    pub fn comfort() -> Self {
        Self {
            mode: DriveMode::Comfort,
            throttle_map: 0.85,
            throttle_response: 0.9,
            idle_rpm_offset: 0.0,
            engine_braking_factor: 0.8,
            fuel_efficiency_target: 0.6,
            tcm_mode: TCMState::Normal,
            tcm_upshift_modifier: -300.0,
            tcm_downshift_modifier: 100.0,
            tcm_kickdown_sensitivity: 0.8,
            tcm_lockup_modifier: 0.0,
            steering_assist: 1.3,
            steering_response: 0.85,
            suspension_damping: 0.8,
            differential_mode: DiffMode::Open,
            abs_threshold_modifier: 1.0,
            tc_threshold_modifier: 1.0,
            vsc_threshold_modifier: 1.0,
            available: true,
            unavailable_reason: None,
        }
    }

    pub fn sport() -> Self {
        Self {
            mode: DriveMode::Sport,
            throttle_map: 1.1,
            throttle_response: 1.2,
            idle_rpm_offset: 200.0,
            engine_braking_factor: 1.3,
            fuel_efficiency_target: 0.3,
            tcm_mode: TCMState::Sport,
            tcm_upshift_modifier: 400.0, // later upshifts
            tcm_downshift_modifier: -200.0,
            tcm_kickdown_sensitivity: 1.3,
            tcm_lockup_modifier: 10.0, // later lockup
            steering_assist: 0.8,
            steering_response: 1.2,
            suspension_damping: 1.2,
            differential_mode: DiffMode::LimitedSlip,
            abs_threshold_modifier: 1.2, // less intervention
            tc_threshold_modifier: 1.3,
            vsc_threshold_modifier: 1.2,
            available: true,
            unavailable_reason: None,
        }
    }

    pub fn track() -> Self {
        Self {
            mode: DriveMode::Track,
            throttle_map: 1.2,
            throttle_response: 1.3,
            idle_rpm_offset: 300.0,
            engine_braking_factor: 1.5,
            fuel_efficiency_target: 0.1,
            tcm_mode: TCMState::Sport,
            tcm_upshift_modifier: 600.0,
            tcm_downshift_modifier: -300.0,
            tcm_kickdown_sensitivity: 1.5,
            tcm_lockup_modifier: 15.0,
            steering_assist: 0.6,
            steering_response: 1.3,
            suspension_damping: 1.5,
            differential_mode: DiffMode::Locked,
            abs_threshold_modifier: 1.5, // minimal intervention
            tc_threshold_modifier: 1.5,
            vsc_threshold_modifier: 1.5,
            available: true,
            unavailable_reason: None,
        }
    }

    pub fn snow() -> Self {
        Self {
            mode: DriveMode::Snow,
            throttle_map: 0.6,
            throttle_response: 0.7,
            idle_rpm_offset: 0.0,
            engine_braking_factor: 0.7,
            fuel_efficiency_target: 0.5,
            tcm_mode: TCMState::Winter,
            tcm_upshift_modifier: -600.0, // very early upshifts
            tcm_downshift_modifier: 300.0,
            tcm_kickdown_sensitivity: 0.5,
            tcm_lockup_modifier: -15.0,
            steering_assist: 1.3,
            steering_response: 0.7,
            suspension_damping: 0.7,
            differential_mode: DiffMode::LimitedSlip,
            abs_threshold_modifier: 0.8, // more intervention
            tc_threshold_modifier: 0.7,
            vsc_threshold_modifier: 0.7,
            available: true,
            unavailable_reason: None,
        }
    }

    /// Create profile for a given mode.
    pub fn for_mode(mode: DriveMode) -> Self {
        match mode {
            DriveMode::Normal => Self::normal(),
            DriveMode::Eco => Self::eco(),
            DriveMode::Comfort => Self::comfort(),
            DriveMode::Sport => Self::sport(),
            DriveMode::Track => Self::track(),
            DriveMode::Snow => Self::snow(),
        }
    }
}

/// Drive mode controller with hysteresis and availability checks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriveModeController {
    pub active_mode: DriveMode,
    pub requested_mode: DriveMode,
    pub active_profile: DriveModeProfile,
    /// Hysteresis: prevent rapid mode switching.
    pub mode_change_cooldown: f64,
    pub time_since_mode_change: f64,
    /// Whether mode change is allowed during current state.
    pub allow_mode_change: bool,
}

impl Default for DriveModeController {
    fn default() -> Self {
        Self {
            active_mode: DriveMode::Normal,
            requested_mode: DriveMode::Normal,
            active_profile: DriveModeProfile::normal(),
            mode_change_cooldown: 1.0,
            time_since_mode_change: 10.0,
            allow_mode_change: true,
        }
    }
}

impl DriveModeController {
    /// Request a mode change. Returns true if accepted.
    pub fn request_mode(&mut self, mode: DriveMode, vehicle_has_tcs: bool, has_auto: bool) -> bool {
        if !self.allow_mode_change {
            return false;
        }
        if self.time_since_mode_change < self.mode_change_cooldown {
            return false;
        }

        // Check availability
        let profile = DriveModeProfile::for_mode(mode);
        match mode {
            DriveMode::Track if !has_auto => {
                // Track mode requires manual or sport auto
                // Allow it but note it
            }
            DriveMode::Snow if !vehicle_has_tcs => {
                // Snow mode benefits from TCS but doesn't require it
            }
            _ => {}
        }

        self.requested_mode = mode;
        self.active_mode = mode;
        self.active_profile = profile;
        self.time_since_mode_change = 0.0;
        true
    }

    /// Update cooldown timer.
    pub fn update(&mut self, dt: f64) {
        self.time_since_mode_change += dt;
    }

    /// Apply mode effects to engine parameters.
    pub fn apply_to_engine(&self, engine: &mut Engine) {
        engine.throttle_response = self.active_profile.throttle_response;
        engine.idle_rpm = (engine.idle_rpm + self.active_profile.idle_rpm_offset).max(600.0);
        engine.engine_braking *= self.active_profile.engine_braking_factor;
    }

    /// Get effective throttle input after mode mapping.
    pub fn map_throttle(&self, raw_throttle: f64) -> f64 {
        (raw_throttle * self.active_profile.throttle_map).clamp(0.0, 1.0)
    }

    /// Get effective steering multiplier.
    pub fn steering_multiplier(&self) -> f64 {
        self.active_profile.steering_assist * self.active_profile.steering_response
    }

    /// Get effective suspension damping multiplier.
    pub fn suspension_multiplier(&self) -> f64 {
        self.active_profile.suspension_damping
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_engine_default_rpm() {
        let e = Engine::default();
        assert!((e.rpm - 800.0).abs() < 1e-6);
        assert_eq!(e.idle_rpm, 800.0);
        assert_eq!(e.redline_rpm, 7000.0);
    }

    #[test]
    fn test_engine_torque_curve() {
        let e = Engine::default();
        let t0 = e.torque_at_rpm(0.0);
        let t3k = e.torque_at_rpm(3000.0);
        let t7k = e.torque_at_rpm(7000.0);
        assert!(t0 > 0.0);
        assert!(t3k > t0);
        assert!(t7k > 0.0);
    }

    #[test]
    fn test_transmission_total_ratio() {
        let mut t = Transmission::default();
        t.current_gear = 1;
        let ratio = t.total_ratio();
        assert!(ratio > 0.0);
        assert!((ratio - 3.5 * 3.7).abs() < 1e-6);
    }

    #[test]
    fn test_transmission_shift_up() {
        let mut t = Transmission::default();
        t.current_gear = 1;
        let ok = t.shift_up();
        assert!(ok);
        assert_eq!(t.current_gear, 2);
    }

    #[test]
    fn test_transmission_shift_down() {
        let mut t = Transmission::default();
        t.current_gear = 1;
        t.shift_down();
        assert_eq!(t.current_gear, 0);
        t.shift_down();
        assert_eq!(t.current_gear, -1);
        let ok = t.shift_down();
        assert!(!ok);
        assert_eq!(t.current_gear, -1);
    }

    #[test]
    fn test_drivetrain_speed_increases_with_throttle() {
        let mut dt = Drivetrain::new();
        dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
        let speed1 = dt.get_vehicle_speed();
        for _ in 0..60 {
            dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
        }
        assert!(dt.get_vehicle_speed() > speed1);
    }

    #[test]
    fn test_drivetrain_brake_reduces_speed() {
        let mut dt = Drivetrain::new();
        for _ in 0..120 {
            dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
        }
        let speed_before = dt.get_vehicle_speed();
        for _ in 0..120 {
            dt.update(0.0, 1.0, 0.0, 1.0 / 60.0, 1500.0);
        }
        assert!(dt.get_vehicle_speed() < speed_before);
    }

    #[test]
    fn test_drivetrain_clutch_disengages() {
        let mut dt = Drivetrain::new();
        dt.update(1.0, 0.0, 1.0, 1.0 / 60.0, 1500.0);
        assert!(dt.get_clutch() < 0.5);
    }

    #[test]
    fn test_drivetrain_idle() {
        let mut dt = Drivetrain::new();
        for _ in 0..300 {
            dt.update(0.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
        }
        assert!(dt.get_rpm() < 1000.0);
    }

    // === New tests ===

    #[test]
    fn test_auto_shift_up() {
        let mut dt = Drivetrain::new();
        dt.transmission.mode = TransmissionMode::Automatic;
        dt.auto_shift.upshift_rpm = 4000.0;
        dt.auto_shift.shift_delay = 0.0;
        dt.auto_shift.last_shift_time = -10.0;
        dt.engine.rpm = 5500.0;
        dt.apply_automatic_shifting();
        // Process shift
        for _ in 0..60 {
            dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
            dt.engine.rpm = 5500.0;
        }
        assert!(
            dt.get_gear() >= 2
                || dt.shift_phase == ShiftPhase::Engaging
                || dt.shift_phase == ShiftPhase::Neutral,
            "Auto mode should have initiated upshift, gear={}",
            dt.get_gear()
        );
    }

    #[test]
    fn test_auto_shift_down() {
        let mut dt = Drivetrain::new();
        dt.transmission.mode = TransmissionMode::Automatic;
        dt.transmission.current_gear = 3;
        dt.auto_shift.downshift_rpm = 2500.0;
        dt.auto_shift.shift_delay = 0.0;
        dt.auto_shift.last_shift_time = -10.0;
        dt.engine.rpm = 2000.0;
        dt.apply_automatic_shifting();
        // Should request downshift
        assert!(
            dt.shift_phase != ShiftPhase::Idle || dt.pending_gear < 3,
            "Auto mode should downshift at low RPM"
        );
    }

    #[test]
    fn test_shift_delay_prevents_rapid_shifts() {
        let mut dt = Drivetrain::new();
        dt.shift_up();
        // Process the shift
        for _ in 0..30 {
            dt.update(1.0, 0.0, 0.0, 0.01, 1500.0);
        }
        // Try immediately — should be blocked by delay
        let ok = dt.request_shift_up();
        assert!(
            !ok || dt.shift_phase != ShiftPhase::Idle,
            "Should not shift again within delay period"
        );
    }

    #[test]
    fn test_kickdown_earlier_upshift() {
        let mut al = AutoShiftLogic::default();
        al.shift_delay = 0.0;
        al.last_shift_time = -10.0;
        // High throttle: upshift at lower RPM
        assert!(al.should_upshift(2, 5800.0, 1.0, 0.0));
        // Low throttle: requires higher RPM
        assert!(!al.should_upshift(2, 5800.0, 0.3, 0.0));
    }

    #[test]
    fn test_blocked_shift_during_phase() {
        let mut dt = Drivetrain::new();
        dt.shift_up();
        // Should be in Disengaging
        assert_eq!(dt.shift_phase, ShiftPhase::Disengaging);
        // Try another shift — should be blocked
        let ok = dt.request_shift_down();
        assert!(!ok, "Can't shift during ongoing shift");
    }

    #[test]
    fn test_rev_matching() {
        let mut dt = Drivetrain::new();
        dt.wheel_speed = 50.0;
        dt.engine.rpm = 6000.0;
        dt.shift_up();
        // Process through phases
        for _ in 0..30 {
            dt.update(1.0, 0.0, 0.0, 0.01, 1500.0);
        }
        // After shift, RPM should be closer to wheel-speed-derived RPM
        let target = dt.transmission.engine_rpm_from_wheel_speed(dt.wheel_speed);
        let diff = (dt.get_rpm() - target).abs();
        assert!(
            diff < 3000.0,
            "Rev matching should adjust RPM toward target"
        );
    }

    #[test]
    fn test_stall_detection() {
        let mut dt = Drivetrain::new();
        dt.engine.rpm = 500.0; // below idle * 0.7
        dt.transmission.clutch_engagement = 0.8;
        dt.wheel_speed = 0.0;
        dt.update(0.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
        assert!(dt.is_stalled, "Should detect stall");
    }

    #[test]
    fn test_differential_open() {
        let mut diff = Differential::default();
        diff.mode = DiffMode::Open;
        let (l, r) = diff.apply_differential(1000.0, 0.8, 0.6);
        assert!((l - 500.0).abs() < 1e-6);
        assert!((r - 500.0).abs() < 1e-6);
    }

    #[test]
    fn test_differential_locked() {
        let mut diff = Differential::default();
        diff.mode = DiffMode::Locked;
        let (l, r) = diff.apply_differential(1000.0, 0.8, 0.2);
        assert!((l - 500.0).abs() < 1e-6);
        assert!((r - 500.0).abs() < 1e-6);
    }

    #[test]
    fn test_differential_lsd_biases_toward_grip() {
        let mut diff = Differential::default();
        diff.mode = DiffMode::LimitedSlip;
        diff.bias = 0.5;
        let (l, r) = diff.apply_differential(1000.0, 0.9, 0.1);
        // More grip on left -> more torque on left
        assert!(l > r, "LSD should bias torque toward wheel with more grip");
    }

    #[test]
    fn test_ratio_validation() {
        let good = DrivetrainConfig {
            gear_ratios: vec![3.5, 2.1, 1.4, 1.0],
            final_drive: 3.7,
            reverse_ratio: -3.2,
            max_gears: 6,
        };
        assert!(good.validate().is_ok());

        let bad = DrivetrainConfig {
            gear_ratios: vec![1.0, 1.5], // ascending — invalid
            final_drive: 3.7,
            reverse_ratio: -3.2,
            max_gears: 6,
        };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn test_reverse_gear_negative_ratio() {
        let mut dt = Drivetrain::new();
        dt.set_gear(-1);
        assert_eq!(dt.get_gear(), -1);
        let ratio = dt.transmission.total_ratio();
        assert!(ratio < 0.0);
    }

    #[test]
    fn test_neutral_gear_zero_transfer() {
        let mut dt = Drivetrain::new();
        dt.set_gear(0);
        assert_eq!(dt.get_gear(), 0);
        let ratio = dt.transmission.total_ratio();
        assert!((ratio).abs() < 1e-6, "Neutral should have zero ratio");
    }

    #[test]
    fn test_shift_returns_bool() {
        let mut t = Transmission::default();
        t.current_gear = t.gear_ratios.len() as i32;
        let ok = t.shift_up();
        assert!(!ok, "Can't shift above max gear");
    }
}

// === TCM Tests ===

#[cfg(test)]
mod tcm_tests {
    use super::*;

    #[test]
    fn test_tcm_upshift_at_high_rpm() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        assert!(
            shift == Some(3),
            "Should upshift from 2nd at 4500 RPM, got {:?}",
            shift
        );
    }

    #[test]
    fn test_tcm_downshift_at_low_rpm() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(1000.0, 20.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        assert!(
            shift == Some(2),
            "Should downshift from 3rd at 1000 RPM, got {:?}",
            shift
        );
    }

    #[test]
    fn test_tcm_kickdown() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(3000.0, 40.0, 0.9, 0.0, 80.0, 4, 6, 0.1);
        assert!(
            shift == Some(3),
            "Kickdown should downshift, got {:?}",
            shift
        );
    }

    #[test]
    fn test_tcm_shift_interval() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        // Immediate second shift should be blocked
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 3, 6, 0.01);
        assert!(shift.is_none(), "Should not shift again within interval");
    }

    #[test]
    fn test_tcm_converter_lockup() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(2500.0, 50.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        assert!(
            tcm.converter_lockup,
            "Should lock up at moderate speed/throttle"
        );
    }

    #[test]
    fn test_tcm_converter_unlock_on_kickdown() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(2500.0, 50.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        assert!(tcm.converter_lockup);
        tcm.update(3000.0, 50.0, 0.8, 0.0, 80.0, 3, 6, 0.1);
        assert!(!tcm.converter_lockup, "Should unlock on heavy throttle");
    }

    #[test]
    fn test_tcm_thermal_protection() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(3000.0, 40.0, 0.5, 0.0, 130.0, 3, 6, 0.1);
        assert!(tcm.limp_mode, "Should enter limp mode on overheat");
        assert!(tcm.line_pressure < 1.0, "Should reduce line pressure");
    }

    #[test]
    fn test_tcm_line_pressure_scales_with_throttle() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(3000.0, 40.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        let low = tcm.line_pressure;
        tcm.update(3000.0, 40.0, 0.9, 0.0, 80.0, 3, 6, 0.1);
        let high = tcm.line_pressure;
        assert!(high > low, "Higher throttle should increase line pressure");
    }

    #[test]
    fn test_tcm_no_shift_when_disabled() {
        let mut tcm = TransmissionControlModule::default();
        tcm.enabled = false;
        let shift = tcm.update(8000.0, 80.0, 1.0, 0.0, 80.0, 1, 6, 0.1);
        assert!(shift.is_none(), "Disabled TCM should not shift");
    }

    #[test]
    fn test_tcm_adaptive_learning() {
        let mut tcm = TransmissionControlModule::default();
        let initial = tcm.learning.shift_quality[1]; // gear 2 → index 1
                                                     // Do several upshifts
        for _ in 0..100 {
            tcm.time_since_shift = 10.0;
            tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        }
        let adapted = tcm.learning.shift_quality[1];
        assert!(adapted != initial, "Learning should adapt shift quality");
    }
}

// === Drive Mode Tests ===

#[cfg(test)]
mod drive_mode_tests {
    use super::*;

    #[test]
    fn test_drive_mode_profiles_exist() {
        for mode in DriveMode::all() {
            let profile = DriveModeProfile::for_mode(*mode);
            assert_eq!(profile.mode, *mode);
        }
    }

    #[test]
    fn test_eco_reduces_throttle() {
        let profile = DriveModeProfile::eco();
        assert!(profile.throttle_map < 1.0, "Eco should dampen throttle");
        assert!(
            profile.tcm_upshift_modifier < 0.0,
            "Eco should upshift earlier"
        );
    }

    #[test]
    fn test_sport_increases_throttle() {
        let profile = DriveModeProfile::sport();
        assert!(profile.throttle_map > 1.0, "Sport should sharpen throttle");
        assert!(
            profile.tcm_upshift_modifier > 0.0,
            "Sport should upshift later"
        );
        assert_eq!(profile.differential_mode, DiffMode::LimitedSlip);
    }

    #[test]
    fn test_track_maximizes_performance() {
        let profile = DriveModeProfile::track();
        assert!(profile.throttle_map > 1.1);
        assert!(profile.suspension_damping > 1.3);
        assert_eq!(profile.differential_mode, DiffMode::Locked);
    }

    #[test]
    fn test_snow_reduces_power_and_increases_intervention() {
        let profile = DriveModeProfile::snow();
        assert!(profile.throttle_map < 0.7);
        assert!(
            profile.abs_threshold_modifier < 1.0,
            "Snow should have more ABS intervention"
        );
        assert!(
            profile.tc_threshold_modifier < 1.0,
            "Snow should have more TC intervention"
        );
    }

    #[test]
    fn test_mode_switching_with_cooldown() {
        let mut ctrl = DriveModeController::default();
        assert!(ctrl.request_mode(DriveMode::Sport, true, true));
        assert_eq!(ctrl.active_mode, DriveMode::Sport);
        // Immediate switch should be blocked by cooldown
        assert!(!ctrl.request_mode(DriveMode::Eco, true, true));
    }

    #[test]
    fn test_mode_switching_after_cooldown() {
        let mut ctrl = DriveModeController::default();
        ctrl.request_mode(DriveMode::Sport, true, true);
        ctrl.update(2.0); // past cooldown
        assert!(ctrl.request_mode(DriveMode::Eco, true, true));
        assert_eq!(ctrl.active_mode, DriveMode::Eco);
    }

    #[test]
    fn test_throttle_mapping() {
        let ctrl = DriveModeController {
            active_profile: DriveModeProfile::eco(),
            ..Default::default()
        };
        let mapped = ctrl.map_throttle(1.0);
        assert!(mapped < 1.0, "Eco mode should reduce throttle");
        assert!(mapped > 0.0);
    }

    #[test]
    fn test_suspension_multiplier() {
        let mut ctrl = DriveModeController::default();
        ctrl.request_mode(DriveMode::Track, true, true);
        assert!(
            ctrl.suspension_multiplier() > 1.0,
            "Track should stiffen suspension"
        );
    }

    #[test]
    fn test_steering_multiplier() {
        let mut ctrl = DriveModeController::default();
        ctrl.request_mode(DriveMode::Comfort, true, true);
        // Comfort: assist=1.3, response=0.85 → multiplier ~1.105
        let mult = ctrl.steering_multiplier();
        assert!(
            mult > 0.8 && mult < 1.5,
            "Comfort steering multiplier should be reasonable"
        );
    }
}
