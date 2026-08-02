use serde::{Deserialize, Serialize};

/// Progressive engine damage tracking.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineDamage {
    pub wear_level: f64,
    pub overheating_cycles: u32,
    pub overrev_cycles: u32,
    pub knock_events: u32,
    pub has_coolant_leak: bool,
    pub has_oil_leak: bool,
    pub head_gasket_failed: bool,
    pub bearing_damage: f64,
    pub is_seized: bool,
    pub is_on_fire: bool,
    pub fire_timer: f64,
    /// Lugging events (high gear, low RPM, high load).
    pub lugging_events: u32,
    /// Causal damage event history (bounded).
    pub damage_history: Vec<DamageEvent>,
}

impl Default for EngineDamage {
    fn default() -> Self {
        Self {
            wear_level: 0.0,
            overheating_cycles: 0,
            overrev_cycles: 0,
            knock_events: 0,
            has_coolant_leak: false,
            has_oil_leak: false,
            head_gasket_failed: false,
            bearing_damage: 0.0,
            is_seized: false,
            is_on_fire: false,
            fire_timer: 0.0,
            lugging_events: 0,
            damage_history: Vec::new(),
        }
    }
}

impl EngineDamage {
    pub fn check_overheat(&mut self, coolant_temp: f64) {
        if coolant_temp > 110.0 {
            self.overheating_cycles += 1;
        }
        if coolant_temp > 125.0 && !self.head_gasket_failed {
            // Accumulated risk — simplified: every 100 cycles of >125°C risks head gasket
            if self.overheating_cycles > 100 {
                self.head_gasket_failed = true;
                self.has_coolant_leak = true;
            }
        }
        if coolant_temp > 140.0 {
            self.wear_level = (self.wear_level + 0.005).min(1.0);
            if self.wear_level > 0.8 && !self.is_on_fire {
                // Fire risk increases with extreme temp
                self.fire_timer += 0.01;
                if self.fire_timer > 1.0 {
                    self.is_on_fire = true;
                }
            }
        }
    }

    pub fn check_overrev(&mut self, rpm: f64, redline: f64) {
        if rpm > redline * 1.1 {
            self.overrev_cycles += 1;
            self.wear_level = (self.wear_level + 0.0001).min(1.0);
        }
    }

    pub fn check_oil_starvation(&mut self, pressure: f64) {
        if pressure < 50.0 {
            self.bearing_damage = (self.bearing_damage + 0.001).min(1.0);
        }
        if pressure < 10.0 {
            self.bearing_damage = (self.bearing_damage + 0.01).min(1.0);
        }
        if self.bearing_damage > 0.8 {
            self.is_seized = true;
        }
    }

    pub fn check_knock(&mut self) {
        self.knock_events += 1;
        if self.knock_events > 50 {
            self.wear_level = (self.wear_level + 0.001).min(1.0);
        }
    }

    /// Check for lugging (high gear, low RPM, high throttle).
    pub fn check_lugging(&mut self, rpm: f64, throttle: f64, gear: i32, time: f64) {
        if gear >= 3 && rpm < 1500.0 && throttle > 0.5 {
            self.lugging_events += 1;
            self.wear_level = (self.wear_level + 0.0002).min(1.0);
            if self.damage_history.len() < 50 {
                self.damage_history.push(DamageEvent {
                    timestamp: time,
                    cause: "lugging".into(),
                    severity: 0.3,
                    subsystem: "engine".into(),
                });
            }
        }
    }

    /// Record a damage event in the causal history.
    pub fn record_damage_event(&mut self, time: f64, cause: &str, severity: f64, subsystem: &str) {
        if self.damage_history.len() < 50 {
            self.damage_history.push(DamageEvent {
                timestamp: time,
                cause: cause.to_string(),
                severity,
                subsystem: subsystem.to_string(),
            });
        }
    }

    pub fn update(&mut self, dt: f64) {
        if self.is_on_fire {
            self.fire_timer += dt;
            self.wear_level = (self.wear_level + 0.05 * dt).min(1.0);
        }
    }
}

/// Staged blow-up outcomes from 0=ok to 5=catastrophic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BlowupStage {
    Ok = 0,
    WarningLamp = 1,
    TorqueDerate = 2,
    RoughRunning = 3,
    SeverePowerLoss = 4,
    SeizureOrFire = 5,
    CompleteFailure = 6,
}

impl BlowupStage {
    pub fn from_level(level: u8) -> Self {
        match level {
            0 => Self::Ok,
            1 => Self::WarningLamp,
            2 => Self::TorqueDerate,
            3 => Self::RoughRunning,
            4 => Self::SeverePowerLoss,
            5 => Self::SeizureOrFire,
            _ => Self::CompleteFailure,
        }
    }
}

/// Causal damage event with timestamp and severity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DamageEvent {
    pub timestamp: f64,
    pub cause: String,
    pub severity: f64,
    pub subsystem: String,
}
