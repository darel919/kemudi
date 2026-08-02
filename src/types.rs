use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

use crate::drivetrain::{Drivetrain, TransmissionControlModule};
use crate::engine::{
    CoolingSystem, EngineDamage, EngineStressAccumulators, EngineThermal, LubricationSystem,
};
use crate::safety::SafetySystemState;
use crate::suspension::{self, SteeringConfig, SuspensionConfig, WheelState};
use crate::tires::{TireState, TireThermalParams};

pub const TELEMETRY_LEN: usize = 80;
pub(crate) const TERRAIN_GRID_SIZE: usize = 64;
pub(crate) const TERRAIN_EXTENT: f64 = 400.0;

// Stable indices used by the worker/UI. Keeping this a fixed-size buffer makes
// telemetry cheap to transfer and avoids serializing large object graphs.
pub const T_SPEED_MPS: usize = 0;
pub const T_SPEED_KMH: usize = 1;
pub const T_RPM: usize = 2;
pub const T_GEAR: usize = 3;
pub const T_THROTTLE: usize = 4;
pub const T_BRAKE: usize = 5;
pub const T_STEERING: usize = 6;
pub const T_CLUTCH: usize = 7;
pub const T_COOLANT: usize = 8;
pub const T_OIL_TEMP: usize = 9;
pub const T_OIL_PRESSURE: usize = 10;
pub const T_TRANS_TEMP: usize = 11;
pub const T_FUEL: usize = 12;
pub const T_DERATE: usize = 13;
pub const T_DAMAGE: usize = 14;
pub const T_SUSPENSION_LOAD: usize = 15;
pub const T_GRIP: usize = 16;
pub const T_TERRAIN_HEIGHT: usize = 17;
pub const T_WHEEL_SPEED: usize = 18;
pub const T_DRIVE_TORQUE: usize = 19;
pub const T_ENGINE_RUNNING: usize = 20;
pub const T_SHIFT_PHASE: usize = 21;
pub const T_ABS_ACTIVE: usize = 22;
pub const T_TC_ACTIVE: usize = 23;
pub const T_VSC_ACTIVE: usize = 24;
pub const T_BROKEN_BEAMS: usize = 25;
pub const T_ENGINE_STAGE: usize = 26;
pub const T_TIRE_WEAR: usize = 27;
pub const T_TIRE_TEMP: usize = 28;
pub const T_CONTACTS: usize = 29;
pub const T_SIM_TIME: usize = 30;
pub const T_FUEL_PERCENT: usize = 31;
pub const T_TCM_STATE: usize = 32;
pub const T_TCM_LINE_PRESSURE: usize = 33;
pub const T_TCM_LOCKUP: usize = 34;
pub const T_REQUESTED_GEAR: usize = 35;
pub const T_SLIP_RATIO: usize = 36;
pub const T_YAW_ERROR: usize = 37;
pub const T_LUGGING_STRESS: usize = 38;
pub const T_OVERREV_STRESS: usize = 39;
pub const T_RUT_DEPTH: usize = 40;
pub const T_INPUT_SHAFT_RPM: usize = 41;
pub const T_OUTPUT_SHAFT_RPM: usize = 42;
pub const T_CONVERTER_COUPLING: usize = 43;
pub const T_WHEEL_SPEED_FL: usize = 44;
pub const T_WHEEL_SPEED_FR: usize = 45;
pub const T_WHEEL_SPEED_RL: usize = 46;
pub const T_WHEEL_SPEED_RR: usize = 47;
pub const T_SUSPENSION_COMPRESSION_FL: usize = 48;
pub const T_SUSPENSION_COMPRESSION_FR: usize = 49;
pub const T_SUSPENSION_COMPRESSION_RL: usize = 50;
pub const T_SUSPENSION_COMPRESSION_RR: usize = 51;
pub const T_SUSPENSION_LOAD_FL: usize = 52;
pub const T_SUSPENSION_LOAD_FR: usize = 53;
pub const T_SUSPENSION_LOAD_RL: usize = 54;
pub const T_SUSPENSION_LOAD_RR: usize = 55;
pub const T_TIRE_TEMP_FL: usize = 56;
pub const T_TIRE_TEMP_FR: usize = 57;
pub const T_TIRE_TEMP_RL: usize = 58;
pub const T_TIRE_TEMP_RR: usize = 59;
pub const T_TIRE_WEAR_FL: usize = 60;
pub const T_TIRE_WEAR_FR: usize = 61;
pub const T_TIRE_WEAR_RL: usize = 62;
pub const T_TIRE_WEAR_RR: usize = 63;
pub const T_IMPACT_SEVERITY: usize = 64;
pub const T_DEFORMATION_DEPTH: usize = 65;
pub const T_BROKEN_PARTS: usize = 66;
pub const T_DAMAGE_ZONE: usize = 67;
pub const T_ENGINE_WARNING: usize = 68;
pub const T_ADAS_FCW: usize = 69;
pub const T_ADAS_AEB: usize = 70;
pub const T_ADAS_CONFIDENCE: usize = 71;
pub const T_TCM_FAULT_MASK: usize = 72;
pub const T_TCM_DIAGNOSTIC_CODE: usize = 73;
pub const T_TCM_SENSOR_INPUT_RPM: usize = 74;
pub const T_TCM_SENSOR_SPEED_KMH: usize = 75;
pub const T_TCM_SENSOR_AGE: usize = 76;
pub const T_TCM_SHIFT_LATENCY: usize = 77;
pub const T_TCM_TORQUE_REDUCTION: usize = 78;
pub const T_TCM_FAIL_SAFE_GEAR: usize = 79;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Node {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub mass: f64,
    pub inv_mass: f64,
    pub fixed: bool,
    pub collision: bool,
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    pub last_force: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Beam {
    pub id: usize,
    pub node_a: usize,
    pub node_b: usize,
    pub stiffness: f64,
    pub damping: f64,
    pub strength: f64,
    pub length: f64,
    pub initial_length: f64,
    pub broken: bool,
    pub lambda: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpactEvent {
    pub timestamp: f64,
    pub severity: f64,
    pub zone: u8,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TriangleConstraint {
    pub(crate) a: usize,
    pub(crate) b: usize,
    pub(crate) c: usize,
    pub(crate) rest_area: f64,
    pub(crate) lambda: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Controls {
    pub(crate) steering: f64,
    pub(crate) throttle: f64,
    pub(crate) brake: f64,
    pub(crate) clutch: f64,
    pub(crate) handbrake: bool,
    pub(crate) gear_up: bool,
    pub(crate) gear_down: bool,
    pub(crate) engine_on: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub nodes: Vec<Node>,
    pub beams: Vec<Beam>,
    pub triangles: Vec<[usize; 3]>,
}

#[wasm_bindgen]
pub struct PhysicsWorld {
    pub(crate) nodes: Vec<Node>,
    pub(crate) beams: Vec<Beam>,
    pub(crate) triangles: Vec<TriangleConstraint>,
    pub(crate) gravity: f64,
    pub(crate) time: f64,
    pub(crate) air_density: f64,
    pub(crate) drag_coefficient: f64,
    pub(crate) frontal_area: f64,
    pub(crate) accumulator: f64,
    pub(crate) fixed_dt: f64,
    pub(crate) max_substeps: u32,
    pub(crate) rest_positions: Vec<[f64; 3]>,
    pub(crate) base_node_masses: Vec<f64>,
    pub(crate) constraint_start_positions: Vec<[f64; 3]>,
    pub(crate) previous_forward_speed: f64,
    pub(crate) previous_yaw: f64,
    pub(crate) yaw_rate: f64,
    pub(crate) controls: Controls,
    pub(crate) terrain_profile: u8,
    pub(crate) terrain_ruts: Vec<f64>,
    /// Base ground friction for chassis-terrain contact. Set from map data,
    /// NOT hardcoded per profile. Computed from terrain layers on the main thread.
    pub(crate) ground_friction: f64,
    /// Surface roughness override from map data (0 = smooth, 1 = rough).
    pub(crate) surface_roughness: f64,
    /// Surface moisture from map data (0 = dry, 1 = wet).
    pub(crate) surface_moisture: f64,
    /// Surface compactness from map data (0 = loose, 1 = compact).
    pub(crate) surface_compactness: f64,
    /// Surface preset index into surface_presets() array.
    pub(crate) surface_preset_index: u8,
    pub(crate) drivetrain: Drivetrain,
    pub(crate) tcm: TransmissionControlModule,
    pub(crate) thermal: EngineThermal,
    pub(crate) cooling: CoolingSystem,
    pub(crate) lubrication: LubricationSystem,
    pub(crate) engine_damage: EngineDamage,
    pub(crate) engine_stress: EngineStressAccumulators,
    pub(crate) steering_config: SteeringConfig,
    pub(crate) steering_angle: f64,
    pub(crate) suspension: SuspensionConfig,
    pub(crate) wheels: Vec<WheelState>,
    pub(crate) wheel_grips: [f64; 4],
    pub(crate) tires: Vec<TireState>,
    pub(crate) tire_thermal: TireThermalParams,
    pub(crate) fuel: suspension::FuelTank,
    pub(crate) safety: SafetySystemState,
    pub(crate) body_damage: f64,
    pub(crate) impact_severity: f64,
    pub(crate) damage_zone: u8,
    pub(crate) impact_history: Vec<ImpactEvent>,
    pub(crate) adas_target_distance: f64,
    pub(crate) adas_target_relative_speed: f64,
    pub(crate) telemetry: [f64; TELEMETRY_LEN],
}
