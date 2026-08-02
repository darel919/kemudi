use super::{DriveMode, DriveModeProfile};
use serde::{Deserialize, Serialize};

/// Deterministic fault channels exposed to scenarios, diagnostics, and the
/// WASM worker. A set bit means the component is faulty; intermittent faults
/// are controlled separately so replaying a seed produces the same symptoms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum TCMFaultKind {
    InputSpeedSensor = 0,
    OutputSpeedSensor = 1,
    ThrottleSignal = 2,
    TemperatureSensor = 3,
    RangeSensor = 4,
    ShiftSolenoid = 5,
    PressureControl = 6,
    HydraulicPressure = 7,
    ConverterLockup = 8,
    Communication = 9,
    Power = 10,
    AdaptationMemory = 11,
}

impl TCMFaultKind {
    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            0 => Self::InputSpeedSensor,
            1 => Self::OutputSpeedSensor,
            2 => Self::ThrottleSignal,
            3 => Self::TemperatureSensor,
            4 => Self::RangeSensor,
            5 => Self::ShiftSolenoid,
            6 => Self::PressureControl,
            7 => Self::HydraulicPressure,
            8 => Self::ConverterLockup,
            9 => Self::Communication,
            10 => Self::Power,
            11 => Self::AdaptationMemory,
            _ => return None,
        })
    }

    pub const fn mask(self) -> u32 {
        1u32 << self as u32
    }

    /// Representative diagnostic identifiers. The dashboard can map these to
    /// manufacturer-specific text without putting strings in the hot path.
    pub const fn diagnostic_code(self) -> u16 {
        match self {
            Self::InputSpeedSensor => 715,
            Self::OutputSpeedSensor => 720,
            Self::ThrottleSignal => 121,
            Self::TemperatureSensor => 711,
            Self::RangeSensor => 705,
            Self::ShiftSolenoid => 750,
            Self::PressureControl => 868,
            Self::HydraulicPressure => 868,
            Self::ConverterLockup => 741,
            Self::Communication => 101,
            Self::Power => 882,
            Self::AdaptationMemory => 607,
        }
    }
}

/// Transmission Control Module for automatic transmissions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmissionControlModule {
    pub enabled: bool,
    pub drive_mode: DriveMode,
    pub state: TCMState,
    pub shift_schedule: TCMShiftSchedule,
    pub adaptive_pressure: [f64; 10],
    pub converter_lockup: bool,
    pub lockup_speed_threshold: f64,
    pub lockup_throttle_threshold: f64,
    pub line_pressure: f64,
    pub thermal_limit: f64,
    pub limp_mode: bool,
    pub limp_gear: i32,
    pub faults: TCMFaults,
    pub learning: TCMLearning,
    pub pending_shift: Option<i32>,
    pub time_since_shift: f64,
    pub min_shift_interval: f64,
    /// Values after the virtual sensors, not the raw physical state.
    pub observed_input_rpm: f64,
    pub observed_output_speed: f64,
    pub observed_throttle: f64,
    pub observed_temperature: f64,
    pub sensor_age: f64,
    pub shift_latency: f64,
    pub torque_reduction_request: f64,
    pub last_diagnostic_code: u16,
    pub fault_clock: f64,
    pub fault_seed: u64,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TCMShiftSchedule {
    pub upshift_rpm: Vec<f64>,
    pub downshift_rpm: Vec<f64>,
    pub throttle_shift_factor: f64,
    pub kickdown_threshold: f64,
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

/// TCM fault tracking. Legacy fields remain available for saved definitions;
/// new faults use the bitmasks so they are cheap to serialize and inspect.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TCMFaults {
    pub solenoid_faults: Vec<usize>,
    pub hydraulic_pressure_loss: bool,
    pub communication_loss: bool,
    pub sensor_faults: Vec<String>,
    pub overheating: bool,
    pub fault_mask: u32,
    pub intermittent_mask: u32,
    pub diagnostic_codes: Vec<u16>,
}

impl TCMFaults {
    pub fn is_active(&self, kind: TCMFaultKind) -> bool {
        self.fault_mask & kind.mask() != 0
            || match kind {
                TCMFaultKind::HydraulicPressure => self.hydraulic_pressure_loss,
                TCMFaultKind::Communication => self.communication_loss,
                _ => false,
            }
    }

    pub fn is_intermittent(&self, kind: TCMFaultKind) -> bool {
        self.intermittent_mask & kind.mask() != 0
    }

    pub fn set(&mut self, kind: TCMFaultKind, active: bool) {
        if active {
            self.fault_mask |= kind.mask();
            if !self.diagnostic_codes.contains(&kind.diagnostic_code()) {
                self.diagnostic_codes.push(kind.diagnostic_code());
            }
        } else {
            self.fault_mask &= !kind.mask();
            self.intermittent_mask &= !kind.mask();
        }
        match kind {
            TCMFaultKind::HydraulicPressure => self.hydraulic_pressure_loss = active,
            TCMFaultKind::Communication => self.communication_loss = active,
            TCMFaultKind::ShiftSolenoid => {
                if active && self.solenoid_faults.is_empty() {
                    self.solenoid_faults.push(0);
                } else if !active {
                    self.solenoid_faults.clear();
                }
            }
            _ => {}
        }
    }

    pub fn set_intermittent(&mut self, kind: TCMFaultKind, intermittent: bool) {
        if intermittent {
            self.intermittent_mask |= kind.mask();
        } else {
            self.intermittent_mask &= !kind.mask();
        }
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TCMLearning {
    pub shift_quality: Vec<f64>,
    pub clutch_wear_factor: f64,
    pub learning_rate: f64,
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
            drive_mode: DriveMode::Normal,
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
            observed_input_rpm: 800.0,
            observed_output_speed: 0.0,
            observed_throttle: 0.0,
            observed_temperature: 20.0,
            sensor_age: 0.0,
            shift_latency: 0.0,
            torque_reduction_request: 0.0,
            last_diagnostic_code: 0,
            fault_clock: 0.0,
            fault_seed: 0x4d45_4d55_4449,
        }
    }
}

impl TransmissionControlModule {
    /// Backwards-compatible update entry point used by drivetrain unit tests.
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
        self.update_with_brake(
            engine_rpm,
            vehicle_speed,
            throttle,
            0.0,
            grade,
            transmission_temp,
            current_gear,
            max_gear,
            dt,
        )
    }

    /// Update the controller from physical signals. Sensor faults are applied
    /// before scheduling; actuator faults are applied to pressure, lockup,
    /// latency, and the returned shift command.
    pub fn update_with_brake(
        &mut self,
        engine_rpm: f64,
        vehicle_speed: f64,
        throttle: f64,
        brake: f64,
        grade: f64,
        transmission_temp: f64,
        current_gear: i32,
        max_gear: i32,
        dt: f64,
    ) -> Option<i32> {
        if !self.enabled {
            self.state = TCMState::Off;
            return None;
        }
        let dt = if dt.is_finite() {
            dt.clamp(0.0, 0.25)
        } else {
            0.0
        };
        self.fault_clock += dt;
        self.time_since_shift += dt;

        let input_rpm = self.sensor_input(engine_rpm, TCMFaultKind::InputSpeedSensor, dt);
        let speed = self.sensor_speed(vehicle_speed, dt);
        let throttle = self.sensor_throttle(throttle, dt);
        let temperature = self.sensor_temperature(transmission_temp, dt);
        self.observed_input_rpm = input_rpm;
        self.observed_output_speed = speed;
        self.observed_throttle = throttle;
        self.observed_temperature = temperature;

        self.check_faults(temperature);
        if !self.limp_mode && self.state != TCMState::Fault {
            self.state = DriveModeProfile::for_mode(self.drive_mode).tcm_mode;
        }
        self.update_line_pressure(throttle, current_gear);
        self.update_converter_lockup(speed, throttle, input_rpm, brake);
        self.shift_latency = self.shift_duration_multiplier();
        self.torque_reduction_request = self.torque_reduction();

        if self.effective_fault(TCMFaultKind::Power)
            || self.effective_fault(TCMFaultKind::Communication)
            || self.effective_fault(TCMFaultKind::RangeSensor)
        {
            self.limp_mode = true;
            self.state = TCMState::Fault;
            self.pending_shift = None;
            self.line_pressure = self.line_pressure.max(0.85);
            return None;
        }

        if self.limp_mode || self.state == TCMState::Fault {
            self.pending_shift = None;
            return None;
        }
        if self.time_since_shift < self.min_shift_interval {
            return None;
        }

        let requested =
            self.determine_shift(input_rpm, speed, throttle, grade, current_gear, max_gear);
        let Some(target) = requested else { return None };
        self.pending_shift = Some(target);
        self.time_since_shift = 0.0;

        // A failed solenoid can leave the currently selected clutch circuit
        // unchanged. The TCM still records the requested gear, but the
        // physical transmission never receives the command.
        if self.effective_fault(TCMFaultKind::ShiftSolenoid)
            || self.effective_fault(TCMFaultKind::PressureControl)
            || self.effective_fault(TCMFaultKind::HydraulicPressure)
        {
            return None;
        }
        self.pending_shift = None;
        Some(target)
    }

    fn sensor_input(&mut self, value: f64, kind: TCMFaultKind, dt: f64) -> f64 {
        if !self.effective_fault(kind) {
            self.sensor_age = 0.0;
            self.observed_input_rpm = value.max(0.0);
            return value.max(0.0);
        }
        self.sensor_age += dt;
        self.record_code(kind);
        if self.sensor_age < 0.35 {
            self.observed_input_rpm.max(0.0)
        } else {
            0.0
        }
    }

    fn sensor_speed(&mut self, value: f64, dt: f64) -> f64 {
        let kind = TCMFaultKind::OutputSpeedSensor;
        if !self.effective_fault(kind) {
            self.observed_output_speed = value.max(0.0);
            return value.max(0.0);
        }
        self.sensor_age += dt;
        self.record_code(kind);
        if self.sensor_age < 0.35 {
            self.observed_output_speed.max(0.0)
        } else {
            0.0
        }
    }

    fn sensor_throttle(&mut self, value: f64, dt: f64) -> f64 {
        let kind = TCMFaultKind::ThrottleSignal;
        if !self.effective_fault(kind) {
            self.observed_throttle = value.clamp(0.0, 1.0);
            return value.clamp(0.0, 1.0);
        }
        self.sensor_age += dt;
        self.record_code(kind);
        // A failed throttle signal falls back to a conservative low load.
        if self.sensor_age < 0.35 {
            self.observed_throttle.clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    fn sensor_temperature(&mut self, value: f64, dt: f64) -> f64 {
        let kind = TCMFaultKind::TemperatureSensor;
        if !self.effective_fault(kind) {
            self.observed_temperature = value;
            return value;
        }
        self.sensor_age += dt;
        self.record_code(kind);
        // A thermistor open circuit commonly reads cold, hiding overheating
        // from the controller until clutch slip exposes it physically.
        if self.sensor_age < 0.35 {
            self.observed_temperature
        } else {
            20.0
        }
    }

    fn effective_fault(&self, kind: TCMFaultKind) -> bool {
        if !self.faults.is_active(kind) {
            return false;
        }
        if !self.faults.is_intermittent(kind) {
            return true;
        }
        // Bounded deterministic dropout: 0.75 s good, 0.25 s failed.
        let phase = (self.fault_clock + self.fault_seed as f64 * 1e-9) % 1.0;
        phase >= 0.75
    }

    fn check_faults(&mut self, temperature: f64) {
        if temperature > self.thermal_limit {
            self.faults.overheating = true;
            self.record_code(TCMFaultKind::HydraulicPressure);
            self.state = TCMState::Limp;
            self.limp_mode = true;
        } else if self.effective_fault(TCMFaultKind::HydraulicPressure) {
            self.state = TCMState::Limp;
        }
        if self.effective_fault(TCMFaultKind::InputSpeedSensor)
            || self.effective_fault(TCMFaultKind::OutputSpeedSensor)
            || self.effective_fault(TCMFaultKind::ThrottleSignal)
            || self.effective_fault(TCMFaultKind::TemperatureSensor)
        {
            self.state = TCMState::Fault;
        }
        for kind in [
            TCMFaultKind::Power,
            TCMFaultKind::Communication,
            TCMFaultKind::RangeSensor,
            TCMFaultKind::ShiftSolenoid,
            TCMFaultKind::PressureControl,
            TCMFaultKind::HydraulicPressure,
            TCMFaultKind::ConverterLockup,
            TCMFaultKind::AdaptationMemory,
        ] {
            if self.effective_fault(kind) {
                self.record_code(kind);
            }
        }
    }

    fn update_converter_lockup(&mut self, speed: f64, throttle: f64, rpm: f64, brake: f64) {
        if self.effective_fault(TCMFaultKind::ConverterLockup) {
            // Even/odd seed selects the common stuck-on versus stuck-off
            // failure, keeping the scenario deterministic without another
            // public configuration field.
            self.converter_lockup = self.fault_seed & 1 == 0;
            return;
        }
        let lockup_speed_threshold = (self.lockup_speed_threshold
            + DriveModeProfile::for_mode(self.drive_mode).tcm_lockup_modifier)
            .max(5.0);
        if speed > lockup_speed_threshold
            && throttle < self.lockup_throttle_threshold
            && brake < 0.1
            && rpm > 1500.0
        {
            self.converter_lockup = true;
        } else if speed < lockup_speed_threshold * 0.7 || throttle > 0.7 || brake > 0.2 {
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
        let mut pressure = base * adaptive;
        if self.effective_fault(TCMFaultKind::HydraulicPressure) {
            pressure *= 0.35;
        }
        if self.effective_fault(TCMFaultKind::PressureControl) {
            let oscillation = (self.fault_clock * 18.0).sin() * 0.25;
            pressure *= (1.0 + oscillation).clamp(0.35, 1.4);
        }
        if self.effective_fault(TCMFaultKind::AdaptationMemory) {
            pressure *= 0.65 + (self.fault_clock * 2.0).sin().abs() * 0.5;
        }
        if self.observed_temperature > self.thermal_limit {
            pressure *=
                (1.0 - (self.observed_temperature - self.thermal_limit) / 30.0).clamp(0.4, 1.0);
        }
        self.line_pressure = pressure.clamp(0.15, 1.0);
    }

    fn determine_shift(
        &mut self,
        engine_rpm: f64,
        vehicle_speed: f64,
        throttle: f64,
        grade: f64,
        current_gear: i32,
        max_gear: i32,
    ) -> Option<i32> {
        if current_gear <= 0 || current_gear > max_gear {
            return None;
        }
        let idx = (current_gear - 1) as usize;
        let profile = DriveModeProfile::for_mode(self.drive_mode);
        let adaptation = if self.effective_fault(TCMFaultKind::AdaptationMemory) {
            (self.fault_clock * 1.7).sin() * 1200.0
        } else {
            0.0
        };
        let upshift_threshold = self
            .shift_schedule
            .upshift_rpm
            .get(idx)
            .copied()
            .unwrap_or(f64::INFINITY)
            + profile.tcm_upshift_modifier
            + throttle * self.shift_schedule.throttle_shift_factor * 1000.0
            - grade * self.shift_schedule.grade_factor * 500.0
            + adaptation;

        // Do not walk an automatic transmission through the gears during the
        // launch phase. A torque converter can flare the engine during a
        // launch, especially on a soft surface or while the driven tires are
        // spinning. Treating that flare as a road-speed upshift request leaves
        // the vehicle in a tall gear with no launch torque. If a launch is
        // already in a tall gear, recover one gear at a time instead.
        if throttle > 0.2 && vehicle_speed < 8.0 {
            return if current_gear > 1 {
                Some(current_gear - 1)
            } else {
                None
            };
        }

        let kickdown_rpm = (upshift_threshold - 500.0).max(0.0);
        let kickdown_threshold = (self.shift_schedule.kickdown_threshold
            / profile.tcm_kickdown_sensitivity.max(0.1))
        .clamp(0.5, 0.95);
        if throttle > kickdown_threshold
            && current_gear > 1
            && vehicle_speed > 1.0
            && engine_rpm.is_finite()
            && engine_rpm < kickdown_rpm
        {
            self.adapt_shift_quality(current_gear, current_gear - 1, false);
            return Some(current_gear - 1);
        }
        if idx < self.shift_schedule.upshift_rpm.len()
            && current_gear < max_gear
            && engine_rpm.is_finite()
            && engine_rpm > upshift_threshold
        {
            self.adapt_shift_quality(current_gear, current_gear + 1, true);
            return Some(current_gear + 1);
        }
        if idx < self.shift_schedule.downshift_rpm.len()
            && current_gear > 1
            && engine_rpm
                < self.shift_schedule.downshift_rpm[idx] + profile.tcm_downshift_modifier
                    - grade * self.shift_schedule.grade_factor * 300.0
        {
            self.adapt_shift_quality(current_gear, current_gear - 1, false);
            return Some(current_gear - 1);
        }
        None
    }

    fn adapt_shift_quality(&mut self, from_gear: i32, _to_gear: i32, is_upshift: bool) {
        let idx = (from_gear - 1).max(0) as usize;
        if idx < self.learning.shift_quality.len() {
            let correction = if is_upshift { 0.01 } else { -0.01 };
            let current = self.learning.shift_quality[idx];
            self.learning.shift_quality[idx] = (current + correction * self.learning.learning_rate)
                .clamp(-self.learning.max_adaptation, self.learning.max_adaptation);
        }
    }

    fn record_code(&mut self, kind: TCMFaultKind) {
        self.last_diagnostic_code = kind.diagnostic_code();
        if !self
            .faults
            .diagnostic_codes
            .contains(&self.last_diagnostic_code)
        {
            self.faults.diagnostic_codes.push(self.last_diagnostic_code);
        }
    }

    pub fn set_drive_mode(&mut self, mode: DriveMode) {
        self.drive_mode = mode;
        if self.enabled && !self.limp_mode && self.state != TCMState::Fault {
            self.state = DriveModeProfile::for_mode(mode).tcm_mode;
        }
    }

    pub fn set_fault(&mut self, kind: TCMFaultKind, active: bool) {
        self.faults.set(kind, active);
        if active {
            self.record_code(kind);
        }
        if !active && self.faults.fault_mask == 0 {
            self.state = if self.enabled {
                DriveModeProfile::for_mode(self.drive_mode).tcm_mode
            } else {
                TCMState::Off
            };
            self.limp_mode = false;
        }
    }

    pub fn set_fault_intermittent(&mut self, kind: TCMFaultKind, active: bool) {
        self.faults.set_intermittent(kind, active);
    }

    pub fn clear_faults(&mut self) {
        self.faults.clear();
        self.state = if self.enabled {
            DriveModeProfile::for_mode(self.drive_mode).tcm_mode
        } else {
            TCMState::Off
        };
        self.limp_mode = false;
        self.pending_shift = None;
        self.last_diagnostic_code = 0;
    }

    pub fn set_fault_seed(&mut self, seed: u64) {
        self.fault_seed = seed;
    }

    pub fn shift_duration_multiplier(&self) -> f64 {
        let mut multiplier: f64 = 1.0;
        if self.effective_fault(TCMFaultKind::ShiftSolenoid) {
            multiplier *= 2.2;
        }
        if self.effective_fault(TCMFaultKind::PressureControl) {
            multiplier *= 1.6;
        }
        if self.effective_fault(TCMFaultKind::HydraulicPressure) {
            multiplier *= 1.8;
        }
        multiplier.clamp(1.0, 4.0)
    }

    pub fn torque_reduction(&self) -> f64 {
        let mut reduction: f64 = if self.limp_mode { 0.35 } else { 0.0 };
        if self.effective_fault(TCMFaultKind::Communication)
            || self.effective_fault(TCMFaultKind::Power)
        {
            reduction = reduction.max(0.55);
        }
        if self.effective_fault(TCMFaultKind::HydraulicPressure) {
            reduction = reduction.max(0.2);
        }
        reduction.clamp(0.0, 0.8)
    }

    pub fn fail_safe_gear(&self, max_gear: i32) -> i32 {
        self.limp_gear.clamp(1, max_gear.max(1))
    }
}
