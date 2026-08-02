use super::{DiffMode, Engine, TCMState};
use serde::{Deserialize, Serialize};

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
