use serde::{Deserialize, Serialize};

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
