//! Engine thermal, lubrication, damage, and telemetry facade.
//!
//! Public exports stay compatible with the original module while each engine
//! subsystem has an explicit implementation home.

mod damage;
mod stress;
mod thermal;
mod update;

pub use damage::{BlowupStage, DamageEvent, EngineDamage};
pub use stress::EngineStressAccumulators;
pub use thermal::{CoolingSystem, EngineThermal, LubricationSystem};
pub use update::{
    update_engine, update_engine_full, update_engine_full_with_cause, EngineTelemetry,
};

#[cfg(test)]
mod tests;
