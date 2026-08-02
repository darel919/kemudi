//! Drivetrain domain facade.
//!
//! The public module path remains stable while engine, gearing, differential,
//! transmission control, and drive-mode concerns live in focused files.

mod core;
mod differential;
mod drive_modes;
mod engine;
mod tcm;
mod transmission;

pub use core::Drivetrain;
pub use differential::{DiffMode, Differential};
pub use drive_modes::{DriveMode, DriveModeController, DriveModeProfile};
pub use engine::Engine;
pub use tcm::{
    TCMFaultKind, TCMFaults, TCMLearning, TCMShiftSchedule, TCMState, TransmissionControlModule,
};
pub use transmission::{
    AutoShiftLogic, ClutchShock, DrivetrainConfig, OverrevCause, ShiftPhase, Transmission,
    TransmissionMode,
};

#[cfg(test)]
mod tests;
