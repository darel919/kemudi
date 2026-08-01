//! Public WASM physics API and domain-module exports.

pub mod drivetrain;
pub mod engine;
pub mod safety;
pub mod suspension;
pub mod terrain_contact;
pub mod tires;

mod math;
mod simulation;
mod telemetry;
mod terrain;
mod types;
mod world;

pub use types::PhysicsWorld;
pub use types::*;

#[cfg(test)]
mod tests;
