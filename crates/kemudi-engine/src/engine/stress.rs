use super::BlowupStage;
use serde::{Deserialize, Serialize};

/// Cumulative stress accumulators with hysteresis and cooldown.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineStressAccumulators {
    /// Thermal stress: temperature-above-limit × duration. Cools when temp is safe.
    pub thermal_stress: f64,
    /// Oil starvation stress: pressure-below-limit × duration.
    pub oil_stress: f64,
    /// Over-rev stress: RPM-above-limit × duration × severity multiplier.
    pub overrev_stress: f64,
    /// Lugging stress: high-gear/low-RPM overload × duration.
    pub lugging_stress: f64,
    /// Accumulator cooldown rates (per second).
    pub thermal_cooldown_rate: f64,
    pub oil_cooldown_rate: f64,
    pub overrev_cooldown_rate: f64,
    pub lugging_cooldown_rate: f64,
    /// Accumulator limits before damage escalation.
    pub thermal_limit: f64,
    pub oil_limit: f64,
    pub overrev_limit: f64,
    pub lugging_limit: f64,
}

impl Default for EngineStressAccumulators {
    fn default() -> Self {
        Self {
            thermal_stress: 0.0,
            oil_stress: 0.0,
            overrev_stress: 0.0,
            lugging_stress: 0.0,
            thermal_cooldown_rate: 0.1,
            oil_cooldown_rate: 0.15,
            overrev_cooldown_rate: 0.05,
            lugging_cooldown_rate: 0.1,
            thermal_limit: 1.0,
            oil_limit: 1.0,
            overrev_limit: 1.0,
            lugging_limit: 1.0,
        }
    }
}

impl EngineStressAccumulators {
    /// Accumulate thermal stress. Positive when temp exceeds limit, cools when below.
    pub fn accumulate_thermal(&mut self, coolant_temp: f64, redline_temp: f64, dt: f64) {
        if coolant_temp > redline_temp {
            let severity = (coolant_temp - redline_temp) / 30.0; // normalized
            self.thermal_stress =
                (self.thermal_stress + severity * dt).min(self.thermal_limit * 2.0);
        } else {
            self.thermal_stress = (self.thermal_stress - self.thermal_cooldown_rate * dt).max(0.0);
        }
    }

    /// Accumulate oil starvation stress.
    pub fn accumulate_oil(&mut self, oil_pressure: f64, nominal_pressure: f64, dt: f64) {
        if oil_pressure < nominal_pressure * 0.3 {
            let severity = (1.0 - oil_pressure / (nominal_pressure * 0.3)).min(2.0);
            self.oil_stress = (self.oil_stress + severity * dt).min(self.oil_limit * 2.0);
        } else {
            self.oil_stress = (self.oil_stress - self.oil_cooldown_rate * dt).max(0.0);
        }
    }

    /// Accumulate over-rev stress with cause-dependent severity.
    pub fn accumulate_overrev(
        &mut self,
        rpm: f64,
        redline: f64,
        is_drivetrain_forced: bool,
        dt: f64,
    ) {
        if rpm > redline {
            let excess = (rpm - redline) / redline;
            let severity_multiplier = if is_drivetrain_forced { 2.0 } else { 1.0 };
            let severity = excess * severity_multiplier;
            self.overrev_stress =
                (self.overrev_stress + severity * dt).min(self.overrev_limit * 2.0);
        } else {
            self.overrev_stress = (self.overrev_stress - self.overrev_cooldown_rate * dt).max(0.0);
        }
    }

    /// Accumulate lugging stress (high gear, low RPM, high load).
    pub fn accumulate_lugging(
        &mut self,
        rpm: f64,
        throttle: f64,
        gear: i32,
        min_lug_rpm: f64,
        dt: f64,
    ) {
        if gear >= 3 && rpm < min_lug_rpm && throttle > 0.5 {
            let severity = throttle * (1.0 - rpm / min_lug_rpm);
            self.lugging_stress =
                (self.lugging_stress + severity * dt).min(self.lugging_limit * 2.0);
        } else {
            self.lugging_stress = (self.lugging_stress - self.lugging_cooldown_rate * dt).max(0.0);
        }
    }

    /// Current blow-up stage based on accumulated stress.
    pub fn blowup_stage(&self) -> BlowupStage {
        let max_stress = self
            .thermal_stress
            .max(self.oil_stress)
            .max(self.overrev_stress)
            .max(self.lugging_stress);
        if max_stress > 1.8 {
            BlowupStage::CompleteFailure
        } else if max_stress > 1.5 {
            BlowupStage::SeizureOrFire
        } else if max_stress > 1.2 {
            BlowupStage::SeverePowerLoss
        } else if max_stress > 0.9 {
            BlowupStage::RoughRunning
        } else if max_stress > 0.5 {
            BlowupStage::TorqueDerate
        } else if max_stress > 0.2 {
            BlowupStage::WarningLamp
        } else {
            BlowupStage::Ok
        }
    }

    /// Derate factor (0-1) from accumulated stress.
    pub fn derate_factor(&self) -> f64 {
        let max_stress = self
            .thermal_stress
            .max(self.oil_stress)
            .max(self.overrev_stress)
            .max(self.lugging_stress);
        (1.0 - (max_stress * 0.4)).clamp(0.0, 1.0)
    }
}
