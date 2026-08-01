use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

pub mod drivetrain;
pub mod engine;
pub mod safety;
pub mod suspension;
pub mod terrain_contact;
pub mod tires;

use drivetrain::{DiffMode, Drivetrain, TransmissionControlModule, TransmissionMode};
use engine::{
    update_engine_full_with_cause, CoolingSystem, EngineDamage, EngineStressAccumulators,
    EngineThermal, LubricationSystem,
};
use safety::SafetySystemState;
use suspension::{raycast_wheel, SteeringConfig, SuspensionConfig, WheelState};
use terrain_contact::{calculate_traction, surface_presets, TerrainContact};
use tires::{update_tire, TireCompound, TireState, TireThermalParams};

pub const TELEMETRY_LEN: usize = 72;
const TERRAIN_GRID_SIZE: usize = 64;
const TERRAIN_EXTENT: f64 = 400.0;

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
struct TriangleConstraint {
    a: usize,
    b: usize,
    c: usize,
    rest_area: f64,
    lambda: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct Controls {
    steering: f64,
    throttle: f64,
    brake: f64,
    clutch: f64,
    handbrake: bool,
    gear_up: bool,
    gear_down: bool,
    engine_on: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub nodes: Vec<Node>,
    pub beams: Vec<Beam>,
    pub triangles: Vec<[usize; 3]>,
}

#[wasm_bindgen]
pub struct PhysicsWorld {
    nodes: Vec<Node>,
    beams: Vec<Beam>,
    triangles: Vec<TriangleConstraint>,
    gravity: f64,
    time: f64,
    air_density: f64,
    drag_coefficient: f64,
    frontal_area: f64,
    accumulator: f64,
    fixed_dt: f64,
    max_substeps: u32,
    constraint_start_positions: Vec<[f64; 3]>,
    previous_forward_speed: f64,
    previous_yaw: f64,
    yaw_rate: f64,
    controls: Controls,
    terrain_profile: u8,
    terrain_ruts: Vec<f64>,
    drivetrain: Drivetrain,
    tcm: TransmissionControlModule,
    thermal: EngineThermal,
    cooling: CoolingSystem,
    lubrication: LubricationSystem,
    engine_damage: EngineDamage,
    engine_stress: EngineStressAccumulators,
    steering_config: SteeringConfig,
    steering_angle: f64,
    suspension: SuspensionConfig,
    wheels: Vec<WheelState>,
    wheel_grips: [f64; 4],
    tires: Vec<TireState>,
    tire_thermal: TireThermalParams,
    fuel: suspension::FuelTank,
    safety: SafetySystemState,
    body_damage: f64,
    impact_severity: f64,
    damage_zone: u8,
    impact_history: Vec<ImpactEvent>,
    adas_target_distance: f64,
    adas_target_relative_speed: f64,
    telemetry: [f64; TELEMETRY_LEN],
}

#[wasm_bindgen]
impl PhysicsWorld {
    #[wasm_bindgen(constructor)]
    pub fn new() -> PhysicsWorld {
        PhysicsWorld {
            nodes: Vec::new(),
            beams: Vec::new(),
            triangles: Vec::new(),
            gravity: -9.81,
            time: 0.0,
            air_density: 1.225,
            drag_coefficient: 0.5,
            frontal_area: 2.2,
            accumulator: 0.0,
            fixed_dt: 1.0 / 60.0,
            max_substeps: 8,
            constraint_start_positions: Vec::new(),
            previous_forward_speed: 0.0,
            previous_yaw: 0.0,
            yaw_rate: 0.0,
            controls: Controls::default(),
            terrain_profile: 0,
            terrain_ruts: vec![0.0; TERRAIN_GRID_SIZE * TERRAIN_GRID_SIZE],
            drivetrain: Drivetrain::default(),
            tcm: TransmissionControlModule::default(),
            thermal: EngineThermal::default(),
            cooling: CoolingSystem::default(),
            lubrication: LubricationSystem::default(),
            engine_damage: EngineDamage::default(),
            engine_stress: EngineStressAccumulators::default(),
            steering_config: SteeringConfig::default(),
            steering_angle: 0.0,
            suspension: SuspensionConfig::default(),
            wheels: vec![WheelState::default(); 4],
            wheel_grips: [0.0; 4],
            tires: vec![TireState::default(); 4],
            tire_thermal: TireThermalParams::default(),
            fuel: suspension::FuelTank::default(),
            safety: SafetySystemState::default(),
            body_damage: 0.0,
            impact_severity: 0.0,
            damage_zone: 0,
            impact_history: Vec::new(),
            adas_target_distance: 0.0,
            adas_target_relative_speed: 0.0,
            telemetry: {
                let mut telemetry = [0.0; TELEMETRY_LEN];
                telemetry[T_DERATE] = 1.0;
                telemetry
            },
        }
    }

    pub fn add_node(&mut self, id: usize, x: f64, y: f64, z: f64, mass: f64, fixed: bool) {
        let safe_mass = if mass.is_finite() && mass > 0.0 {
            mass
        } else {
            1.0
        };
        let inv_mass = if fixed { 0.0 } else { 1.0 / safe_mass };
        let safe_x = finite_or_zero(x);
        let safe_y = finite_or_zero(y);
        let safe_z = finite_or_zero(z);
        self.nodes.push(Node {
            id,
            x: safe_x,
            y: safe_y,
            z: safe_z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            mass: safe_mass,
            inv_mass,
            fixed,
            // The first four nodes are suspension mounts, not rigid terrain
            // colliders. Upper-cage nodes may still collide after a severe
            // deformation without pinning a wheel mount to the ground.
            collision: !fixed && self.nodes.len() >= 4,
            fx: 0.0,
            fy: 0.0,
            fz: 0.0,
            last_force: 0.0,
        });
        self.constraint_start_positions
            .push([safe_x, safe_y, safe_z]);
    }

    /// Apply the authored collision flag after node IDs have been mapped to
    /// their runtime indices. The first four nodes are always suspension
    /// mounts, so they remain raycast contacts rather than rigid terrain
    /// colliders.
    pub fn set_node_collision(&mut self, node_id: usize, collision: bool) {
        if let Some(node) = self.nodes.get_mut(node_id) {
            node.collision = node_id >= 4 && !node.fixed && collision;
        }
    }

    pub fn add_beam(
        &mut self,
        id: usize,
        node_a: usize,
        node_b: usize,
        stiffness: f64,
        damping: f64,
        strength: f64,
    ) {
        let length = if let (Some(a), Some(b)) = (self.nodes.get(node_a), self.nodes.get(node_b)) {
            distance(a.x, a.y, a.z, b.x, b.y, b.z).max(1e-4)
        } else {
            1.0
        };
        self.beams.push(Beam {
            id,
            node_a,
            node_b,
            stiffness: stiffness.max(1.0),
            damping: damping.max(0.0),
            strength: strength.max(0.0),
            length,
            initial_length: length,
            broken: false,
            lambda: 0.0,
        });
    }

    pub fn add_triangle(&mut self, a: usize, b: usize, c: usize) {
        if a >= self.nodes.len() || b >= self.nodes.len() || c >= self.nodes.len() {
            return;
        }
        let area = triangle_area(&self.nodes[a], &self.nodes[b], &self.nodes[c]);
        if area > 1e-8 {
            self.triangles.push(TriangleConstraint {
                a,
                b,
                c,
                rest_area: area,
                lambda: 0.0,
            });
        }
    }

    pub fn configure_runtime(
        &mut self,
        idle_rpm: f64,
        redline_rpm: f64,
        limiter_rpm: f64,
        throttle_response: f64,
        engine_braking: f64,
        torque_rpms: &[f64],
        torque_values: &[f64],
        gear_ratios: &[f64],
        final_drive: f64,
        reverse_ratio: f64,
        transmission_mode: u8,
        shift_delay: f64,
        differential_mode: u8,
        differential_bias: f64,
        wheel_spring_rates: &[f64],
        wheel_dampings: &[f64],
        wheel_rebound_dampings: &[f64],
        wheel_rest_lengths: &[f64],
        wheel_travels: &[f64],
        wheel_radii: &[f64],
        tire_compounds: &[u8],
        tire_pressures: &[f64],
        fuel_capacity: f64,
        fuel_consumption: f64,
        idle_consumption: f64,
        abs_enabled: bool,
        traction_control_enabled: bool,
        vsc_enabled: bool,
        adas_forward_collision_warning: bool,
        adas_automatic_emergency_braking: bool,
    ) {
        let engine = &mut self.drivetrain.engine;
        engine.idle_rpm = sane_or(idle_rpm, 800.0).max(100.0);
        engine.redline_rpm = sane_or(redline_rpm, 7000.0).max(engine.idle_rpm + 100.0);
        engine.rev_limiter_rpm =
            sane_or(limiter_rpm, engine.redline_rpm + 200.0).max(engine.redline_rpm);
        engine.throttle_response = sane_or(throttle_response, 0.8).max(0.05);
        engine.engine_braking = sane_or(engine_braking, 30.0).max(0.0);
        if torque_rpms.len() == torque_values.len() && !torque_rpms.is_empty() {
            let mut curve: Vec<(f64, f64)> = torque_rpms
                .iter()
                .zip(torque_values.iter())
                .filter_map(|(rpm, torque)| {
                    if rpm.is_finite() && torque.is_finite() {
                        Some((*rpm, *torque))
                    } else {
                        None
                    }
                })
                .collect();
            curve.sort_by(|a, b| a.0.total_cmp(&b.0));
            if !curve.is_empty() {
                engine.torque_curve = curve;
            }
        }

        let transmission = &mut self.drivetrain.transmission;
        let ratios: Vec<f64> = gear_ratios
            .iter()
            .copied()
            .filter(|r| r.is_finite() && *r > 0.0)
            .collect();
        if !ratios.is_empty() {
            transmission.gear_ratios = ratios;
        }
        transmission.final_drive = sane_or(final_drive, 3.7).abs().max(0.1);
        transmission.reverse_ratio = sane_or(reverse_ratio, -3.2).min(-0.1);
        transmission.mode = if transmission_mode == 1 {
            TransmissionMode::Automatic
        } else {
            TransmissionMode::Manual
        };
        self.tcm.enabled = transmission.mode == TransmissionMode::Automatic;
        self.drivetrain.shift_duration = sane_or(shift_delay, 0.15).clamp(0.05, 1.0);
        self.drivetrain.auto_shift.shift_delay = self.drivetrain.shift_duration;
        self.drivetrain.differential.mode = match differential_mode {
            1 => DiffMode::Locked,
            2 => DiffMode::LimitedSlip,
            _ => DiffMode::Open,
        };
        self.drivetrain.differential.bias = sane_or(differential_bias, 0.5).clamp(0.0, 1.0);

        for i in 0..4 {
            let wheel = self.suspension.wheels.get_mut(i).unwrap();
            wheel.spring_rate = value_or(wheel_spring_rates, i, wheel.spring_rate).max(1.0);
            wheel.damping = value_or(wheel_dampings, i, wheel.damping).max(0.0);
            wheel.rebound_damping =
                value_or(wheel_rebound_dampings, i, wheel.rebound_damping).max(0.0);
            wheel.rest_length = value_or(wheel_rest_lengths, i, wheel.rest_length).max(0.05);
            wheel.travel = value_or(wheel_travels, i, wheel.travel).max(0.02);
            wheel.tire_radius = value_or(wheel_radii, i, wheel.tire_radius).max(0.05);
            let tire = self.tires.get_mut(i).unwrap();
            tire.compound = match value_or_u8(tire_compounds, i, 1) {
                0 => TireCompound::Sport,
                2 => TireCompound::Offroad,
                3 => TireCompound::Mud,
                4 => TireCompound::Snow,
                _ => TireCompound::Street,
            };
            tire.nominal_pressure = value_or(tire_pressures, i, tire.nominal_pressure).max(1.0);
            tire.pressure = tire.nominal_pressure;
            self.wheels[i] = WheelState::default();
        }
        let configured_radius =
            (self.suspension.wheels[2].tire_radius + self.suspension.wheels[3].tire_radius) * 0.5;
        self.drivetrain.set_wheel_radius(configured_radius);
        self.fuel.capacity = sane_or(fuel_capacity, 60.0).max(1.0);
        self.fuel.current_level = self.fuel.capacity;
        self.fuel.base_consumption_rate = sane_or(fuel_consumption, 0.01).max(0.0);
        self.fuel.idle_consumption_rate = sane_or(idle_consumption, 0.0005).max(0.0);
        self.previous_forward_speed = 0.0;
        self.previous_yaw = 0.0;
        self.yaw_rate = 0.0;
        self.safety.abs.enabled = abs_enabled;
        self.safety.traction_control.enabled = traction_control_enabled;
        self.safety.vsc_esc.enabled = vsc_enabled;
        self.safety.adas.forward_collision_warning = adas_forward_collision_warning;
        self.safety.adas.automatic_emergency_braking = adas_automatic_emergency_braking;
    }

    /// Change the driver-selectable gearbox mode without rebuilding the
    /// vehicle. Mode 0 is manual sequential; mode 1 is torque-converter
    /// automatic. A mode change cancels an in-progress shift so the new mode
    /// starts from one authoritative gear state.
    pub fn set_transmission_mode(&mut self, mode: u8) {
        let next = if mode == 1 {
            TransmissionMode::Automatic
        } else {
            TransmissionMode::Manual
        };
        if self.drivetrain.transmission.mode == next {
            return;
        }
        self.drivetrain.transmission.mode = next;
        self.tcm.enabled = next == TransmissionMode::Automatic;
        self.drivetrain.shift_phase = drivetrain::ShiftPhase::Idle;
        self.drivetrain.shift_timer = 0.0;
        self.drivetrain.pending_gear = self.drivetrain.transmission.current_gear;
        self.tcm.converter_lockup = false;
    }

    /// Apply the vehicle's configured automatic shift points to the TCM.
    /// Missing/invalid values retain the safe built-in schedule.
    pub fn set_automatic_shift_schedule(&mut self, upshift_rpm: f64, downshift_rpm: f64) {
        if !upshift_rpm.is_finite() || !downshift_rpm.is_finite() || downshift_rpm >= upshift_rpm {
            return;
        }
        let max_gear = self.drivetrain.transmission.gear_ratios.len();
        self.tcm.shift_schedule.upshift_rpm = vec![upshift_rpm; max_gear];
        self.tcm.shift_schedule.downshift_rpm = vec![downshift_rpm; max_gear];
    }

    pub fn set_controls(
        &mut self,
        steering: f64,
        throttle: f64,
        brake: f64,
        clutch: f64,
        handbrake: bool,
        gear_up: bool,
        gear_down: bool,
        engine_on: bool,
    ) {
        self.controls = Controls {
            steering: sane_or(steering, 0.0).clamp(-1.0, 1.0),
            throttle: sane_or(throttle, 0.0).clamp(0.0, 1.0),
            brake: sane_or(brake, 0.0).clamp(0.0, 1.0),
            clutch: sane_or(clutch, 0.0).clamp(0.0, 1.0),
            handbrake,
            gear_up,
            gear_down,
            engine_on,
        };
    }

    /// 0 = flat asphalt, 1 = bumpy asphalt, 2 = offroad dirt.
    pub fn set_terrain_profile(&mut self, profile: u8) {
        self.terrain_profile = profile.min(2);
        self.terrain_ruts.fill(0.0);
    }

    pub fn set_adas_target(&mut self, distance: f64, relative_speed: f64) {
        self.adas_target_distance = sane_or(distance, 0.0).max(0.0);
        self.adas_target_relative_speed = sane_or(relative_speed, 0.0);
    }

    pub fn add_vehicle(&mut self, _x: f64, _y: f64, _z: f64) -> usize {
        self.nodes.len()
    }

    pub fn step(&mut self, dt: f64) {
        let frame_dt = if dt.is_finite() {
            dt.clamp(0.0, 0.25)
        } else {
            0.0
        };
        self.accumulator += frame_dt;
        let mut substeps = 0;
        while self.accumulator + 1e-12 >= self.fixed_dt && substeps < self.max_substeps {
            self.step_fixed();
            self.accumulator -= self.fixed_dt;
            self.time += self.fixed_dt;
            substeps += 1;
        }
        if substeps == self.max_substeps {
            self.accumulator = self.accumulator.min(self.fixed_dt);
        }
    }

    pub fn get_positions_flat(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.nodes.len() * 3);
        for n in &self.nodes {
            out.extend_from_slice(&[n.x, n.y, n.z]);
        }
        out
    }

    pub fn get_velocities_flat(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.nodes.len() * 3);
        for n in &self.nodes {
            out.extend_from_slice(&[n.vx, n.vy, n.vz]);
        }
        out
    }

    pub fn get_telemetry_flat(&self) -> Vec<f64> {
        self.telemetry.to_vec()
    }

    pub fn apply_force(&mut self, node_id: usize, fx: f64, fy: f64, fz: f64) {
        if let Some(node) = self.nodes.get_mut(node_id) {
            if !node.fixed {
                node.fx += finite_or_zero(fx);
                node.fy += finite_or_zero(fy);
                node.fz += finite_or_zero(fz);
            }
        }
    }

    pub fn get_node_count(&self) -> usize {
        self.nodes.len()
    }
    pub fn get_beam_count(&self) -> usize {
        self.beams.len()
    }
    pub fn get_time(&self) -> f64 {
        self.time
    }
}

impl PhysicsWorld {
    fn step_fixed(&mut self) {
        if self.constraint_start_positions.len() != self.nodes.len() {
            self.constraint_start_positions
                .resize(self.nodes.len(), [0.0; 3]);
        }
        self.apply_vehicle_forces();
        self.apply_forces();
        self.integrate_velocities(self.fixed_dt);
        // Save the unconstrained predicted positions. Constraint projection
        // is a position correction; retaining this state lets us add only the
        // correction back into velocity without discarding force and damping
        // integration from the current step.
        for (start, node) in self.constraint_start_positions.iter_mut().zip(&self.nodes) {
            *start = [node.x, node.y, node.z];
        }
        self.solve_xpbd_constraints();
        self.collide_with_terrain();
        self.apply_drag();
        self.update_telemetry();
    }

    fn apply_vehicle_forces(&mut self) {
        let count = self.nodes.len();
        if count == 0 {
            return;
        }
        let dynamic_nodes = self.nodes.iter().filter(|n| !n.fixed).count().max(1) as f64;
        let avg_vz = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.vz)
            .sum::<f64>()
            / dynamic_nodes;
        let avg_vx = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.vx)
            .sum::<f64>()
            / dynamic_nodes;
        let speed = (avg_vx * avg_vx + avg_vz * avg_vz).sqrt();
        let mass = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.mass)
            .sum::<f64>()
            .max(1.0);
        let radius = if self.suspension.wheels.len() >= 4 {
            (self.suspension.wheels[2].tire_radius + self.suspension.wheels[3].tire_radius) * 0.5
        } else {
            self.suspension
                .wheels
                .first()
                .map(|w| w.tire_radius)
                .unwrap_or(0.3)
        }
        .max(0.05);
        let driven_vz = match (self.nodes.get(2), self.nodes.get(3)) {
            (Some(left), Some(right)) => (left.vz + right.vz) * 0.5,
            _ => avg_vz,
        };
        // The authored vehicle coordinate system points forward along -Z.
        // Keep drivetrain speed in vehicle coordinates so positive torque,
        // engine braking, and reverse all have consistent signs.
        let forward_velocity = -driven_vz;
        let engine_running = self.controls.engine_on
            && self.fuel.current_level > 0.0
            && !self.engine_damage.is_seized;
        if !engine_running {
            // An unpowered wheel free-rolls with the chassis. Do not retain a
            // stale wheel spin and feed it into ABS/traction control.
            self.drivetrain.wheel_speed = forward_velocity / radius;
            self.drivetrain.vehicle_speed = forward_velocity;
        }
        self.update_yaw_rate();

        self.steering_angle = suspension::update_steering(
            &self.steering_config,
            self.controls.steering,
            speed,
            self.steering_angle,
            self.fixed_dt,
        );
        // Build the body frame before the wheel-speed safety controllers run
        // so ABS/TCS see velocity along each steered wheel's rolling axis,
        // rather than only the world Z component.
        let (forward_x, forward_z) = match (
            self.nodes.get(0),
            self.nodes.get(1),
            self.nodes.get(2),
            self.nodes.get(3),
        ) {
            (Some(front_left), Some(front_right), Some(rear_left), Some(rear_right)) => {
                let dx = (front_left.x + front_right.x - rear_left.x - rear_right.x) * 0.5;
                let dz = (front_left.z + front_right.z - rear_left.z - rear_right.z) * 0.5;
                let length = (dx * dx + dz * dz).sqrt();
                if length > 1e-6 {
                    (dx / length, dz / length)
                } else {
                    (0.0, -1.0)
                }
            }
            _ => (0.0, -1.0),
        };
        let right_x = -forward_z;
        let right_z = forward_x;
        let (left_steer_angle, right_steer_angle) = suspension::ackermann_angles(
            self.steering_angle,
            self.steering_config.wheelbase.max(0.1),
            self.steering_config.track_width.max(0.1),
        );
        let mut wheel_speeds = [0.0; 4];
        for i in 0..4 {
            let steering_angle = match i {
                0 => left_steer_angle,
                1 => right_steer_angle,
                _ => 0.0,
            };
            let wheel_cos = steering_angle.cos();
            let wheel_sin = steering_angle.sin();
            let wheel_forward = [
                forward_x * wheel_cos + right_x * wheel_sin,
                forward_z * wheel_cos + right_z * wheel_sin,
            ];
            wheel_speeds[i] = if i >= 2 && engine_running {
                (self.drivetrain.wheel_speed * self.suspension.wheels[i].tire_radius).abs()
            } else {
                self.nodes
                    .get(i)
                    .map(|node| (node.vx * wheel_forward[0] + node.vz * wheel_forward[1]).abs())
                    .unwrap_or(0.0)
            };
        }
        let tc_modifier = self.safety.traction_control.update(
            wheel_speeds,
            speed,
            &[2, 3],
            self.controls.throttle,
            self.fixed_dt,
        );
        let brake_pressures =
            self.safety
                .abs
                .update(wheel_speeds, speed, self.controls.brake, self.fixed_dt);
        self.safety.adas.update_forward_collision(
            self.adas_target_distance,
            self.adas_target_relative_speed,
            speed,
            self.fixed_dt,
        );
        let adas_brake = self.safety.adas.get_aeb_brake(1.0);
        let brake_command = (brake_pressures.iter().sum::<f64>() / brake_pressures.len() as f64
            + adas_brake)
            .clamp(0.0, 1.0);
        let (vsc_brakes, vsc_torque_modifier) = self.safety.vsc_esc.update(
            self.yaw_rate,
            self.steering_angle * speed / self.steering_config.wheelbase.max(0.1),
            self.steering_angle,
            0.0,
            speed,
            wheel_speeds,
            self.fixed_dt,
        );
        let yaw_error =
            self.yaw_rate - self.steering_angle * speed / self.steering_config.wheelbase.max(0.1);
        self.telemetry[T_YAW_ERROR] = yaw_error.clamp(-10.0, 10.0);
        let throttle = if engine_running {
            self.controls.throttle * tc_modifier
        } else {
            0.0
        };
        if engine_running {
            if self.drivetrain.transmission.mode == TransmissionMode::Automatic {
                let coupling = if self.tcm.converter_lockup {
                    1.0
                } else {
                    (0.58 + self.tcm.line_pressure * 0.22).clamp(0.5, 0.85)
                };
                let torque_multiplier = if self.tcm.converter_lockup {
                    1.0
                } else {
                    (1.0 + (1.0 - coupling) * 0.25).clamp(1.0, 1.2)
                };
                self.drivetrain
                    .set_torque_converter_state(coupling, torque_multiplier);
            } else {
                self.drivetrain.set_torque_converter_state(1.0, 1.0);
            }
            if self.drivetrain.transmission.mode == TransmissionMode::Manual
                && self.controls.gear_up
            {
                self.drivetrain.request_shift_up();
            }
            if self.drivetrain.transmission.mode == TransmissionMode::Manual
                && self.controls.gear_down
            {
                let target = self.drivetrain.transmission.current_gear - 1;
                if !self.drivetrain.check_downshift_overrev(target).1 {
                    self.drivetrain.request_shift_down();
                }
            }
            self.drivetrain.update(
                throttle,
                self.controls.brake,
                self.controls.clutch,
                self.fixed_dt,
                mass,
            );
            if self.drivetrain.transmission.mode == TransmissionMode::Automatic {
                let current_gear = self.drivetrain.transmission.current_gear;
                if let Some(target_gear) = self.tcm.update(
                    self.drivetrain.engine.rpm,
                    speed * 3.6,
                    throttle,
                    0.0,
                    self.telemetry[T_TRANS_TEMP],
                    current_gear,
                    self.drivetrain.transmission.gear_ratios.len() as i32,
                    self.fixed_dt,
                ) {
                    if target_gear > current_gear {
                        self.drivetrain.request_shift_up();
                    } else if target_gear < current_gear
                        && !self.drivetrain.check_downshift_overrev(target_gear).1
                    {
                        self.drivetrain.request_shift_down();
                    }
                }
            }
        } else {
            self.drivetrain.set_torque_converter_state(1.0, 1.0);
            self.drivetrain.last_drive_torque = 0.0;
            self.drivetrain.wheel_speed = forward_velocity / radius;
            self.drivetrain.vehicle_speed = forward_velocity;
            self.drivetrain.engine.rpm = 0.0;
        }

        let mut suspension_load = 0.0;
        let mut grip_sum = 0.0;
        let mut terrain_height = 0.0;
        let mut contacts: f64 = 0.0;
        let mut rear_grip = [0.0; 2];
        let mut rear_load = [0.0; 2];
        let mut grounded = [false; 4];
        let mut wheel_forward_velocity = [0.0; 4];
        let mut wheel_lateral_force = [0.0; 4];
        let mut wheel_forward_axes = [[0.0, -1.0]; 4];
        let mut average_slip_ratio = 0.0;
        self.wheel_grips = [0.0; 4];

        for i in 0..4.min(count) {
            let node = self.nodes[i];
            let height = self.terrain_height(node.x, node.z);
            let (force, compression, airborne) = raycast_wheel(
                &self.suspension.wheels[i],
                [node.x, node.y, node.z],
                node.vy,
                height,
            );
            let bump_stop_force = if !airborne && compression > 0.9 {
                let bump_deflection =
                    (compression - 0.9).clamp(0.0, 0.1) * self.suspension.wheels[i].travel.max(0.0);
                self.suspension.bump_stop_rate.max(0.0) * bump_deflection
            } else {
                0.0
            };
            let suspension_force = force.force + bump_stop_force;
            self.wheels[i].compression = compression;
            self.wheels[i].load = suspension_force;
            self.wheels[i].is_airborne = airborne;
            self.wheels[i].contact_point = force.contact_point;
            self.wheels[i].suspension_force = suspension_force;
            let wheel_radius = self.suspension.wheels[i].tire_radius;
            let steering_angle = match i {
                0 => left_steer_angle,
                1 => right_steer_angle,
                _ => 0.0,
            };
            let wheel_cos = steering_angle.cos();
            let wheel_sin = steering_angle.sin();
            let wheel_forward = [
                forward_x * wheel_cos + right_x * wheel_sin,
                forward_z * wheel_cos + right_z * wheel_sin,
            ];
            let wheel_right = [
                right_x * wheel_cos - forward_x * wheel_sin,
                right_z * wheel_cos - forward_z * wheel_sin,
            ];
            wheel_forward_axes[i] = wheel_forward;
            let ground_forward_velocity = node.vx * wheel_forward[0] + node.vz * wheel_forward[1];
            let ground_lateral_velocity = node.vx * wheel_right[0] + node.vz * wheel_right[1];
            wheel_forward_velocity[i] = ground_forward_velocity;
            self.wheels[i].angular_speed = if i >= 2 && engine_running {
                self.drivetrain.wheel_speed
            } else {
                ground_forward_velocity / wheel_radius
            };
            if !airborne {
                grounded[i] = true;
                contacts += 1.0;
                suspension_load += suspension_force;
                terrain_height += height;
                let mut contact = self.terrain_contact(node.x, node.z);
                contact.rut_depth = self.rut_depth_at(node.x, node.z);
                let driven_wheel_velocity = if i >= 2 {
                    self.drivetrain.wheel_speed * wheel_radius
                } else {
                    ground_forward_velocity
                };
                let slip_ratio = (driven_wheel_velocity - ground_forward_velocity)
                    / ground_forward_velocity.abs().max(1.0);
                let slip_angle =
                    ground_lateral_velocity.atan2(ground_forward_velocity.abs().max(1.0));
                average_slip_ratio += slip_ratio.abs();
                let traction = calculate_traction(
                    &contact,
                    slip_ratio,
                    slip_angle,
                    suspension_force,
                    driven_wheel_velocity / wheel_radius,
                );
                let tire_grip_factor = {
                    let tire = &mut self.tires[i];
                    update_tire(
                        tire,
                        &self.tire_thermal,
                        slip_ratio,
                        slip_angle,
                        suspension_force,
                        contact.surface.roughness,
                        25.0,
                        self.fixed_dt,
                    );
                    tire.grip_factor
                };
                let grip = (traction.friction_coefficient
                    + tire_grip_factor * contact.surface.base_friction)
                    .clamp(0.0, 1.5);
                let lateral_capacity = (traction.lateral_grip
                    + tire_grip_factor
                        * contact.surface.base_friction
                        * (1.0 - slip_ratio.abs().min(1.0)))
                .clamp(0.0, 1.5);
                let lateral_demand = (ground_lateral_velocity / speed.max(1.0)).clamp(-1.0, 1.0);
                let lateral_force = -lateral_demand * suspension_force * lateral_capacity;
                let rolling_force =
                    -ground_forward_velocity.signum() * traction.rolling_resistance_force;
                self.wheel_grips[i] = grip;
                wheel_lateral_force[i] = lateral_force;
                grip_sum += grip;
                self.deposit_rut(
                    node.x,
                    node.z,
                    suspension_force,
                    slip_ratio,
                    contact.surface.deformability,
                );
                if i >= 2 {
                    rear_grip[i - 2] = grip;
                    rear_load[i - 2] = suspension_force;
                }
                self.apply_force(
                    i,
                    lateral_force * wheel_right[0] + rolling_force * wheel_forward[0],
                    suspension_force,
                    lateral_force * wheel_right[1] + rolling_force * wheel_forward[1],
                );
            }
        }
        // Anti-roll bars are rated in N/m. Convert normalized suspension
        // compression back to a physical deflection before transferring load
        // between the two sides of an axle. Applying the rate directly to the
        // normalized value makes the bar several times too stiff and can
        // inject vertical energy into the deformable cage.
        for (left, right) in [(0usize, 1usize), (2usize, 3usize)] {
            if grounded[left] && grounded[right] {
                let travel = (self.suspension.wheels[left].travel
                    + self.suspension.wheels[right].travel)
                    * 0.5;
                let raw_transfer = self.suspension.anti_roll_bar_stiffness.max(0.0)
                    * (self.wheels[left].compression - self.wheels[right].compression)
                    * travel.max(0.02);
                let transfer = raw_transfer.clamp(
                    -self.wheels[right].load.max(0.0),
                    self.wheels[left].load.max(0.0),
                );
                self.wheels[left].load = (self.wheels[left].load - transfer).max(0.0);
                self.wheels[right].load += transfer;
                self.wheels[left].suspension_force = self.wheels[left].load;
                self.wheels[right].suspension_force = self.wheels[right].load;
                self.apply_force(left, 0.0, -transfer, 0.0);
                self.apply_force(right, 0.0, transfer, 0.0);
            }
        }
        rear_load[0] = self.wheels[2].load;
        rear_load[1] = self.wheels[3].load;
        let drive_torque =
            self.drivetrain.last_drive_torque * self.engine_derate() * vsc_torque_modifier;
        let (left_torque, right_torque) = self.drivetrain.differential.apply_differential(
            drive_torque,
            rear_grip[0].max(0.01),
            rear_grip[1].max(0.01),
        );
        for (i, torque, grip, load) in [
            (2usize, left_torque, rear_grip[0], rear_load[0]),
            (3usize, right_torque, rear_grip[1], rear_load[1]),
        ] {
            if i < count && grounded[i] && load > 0.0 {
                let wheel_radius = self.suspension.wheels[i].tire_radius;
                let traction_limit = load * grip.max(0.0);
                // Lateral tire force consumes part of the friction circle,
                // leaving the remainder for longitudinal drive torque.
                let lateral_force = wheel_lateral_force[i].abs();
                let longitudinal_limit = (traction_limit * traction_limit
                    - lateral_force * lateral_force)
                    .max(0.0)
                    .sqrt();
                let force_forward =
                    (torque / wheel_radius).clamp(-longitudinal_limit, longitudinal_limit);
                self.apply_force(
                    i,
                    force_forward * wheel_forward_axes[i][0],
                    0.0,
                    force_forward * wheel_forward_axes[i][1],
                );
            }
        }
        // VSC returns wheel brake torques, not just a diagnostic flag. Feed
        // those actuator commands back through the same grounded wheel force
        // path so ESC cannot create a yaw correction without physical load.
        if speed > 0.1 {
            for i in 0..4.min(count) {
                if grounded[i] {
                    let wheel_radius = self.suspension.wheels[i].tire_radius;
                    let vsc_force = (vsc_brakes[i] / wheel_radius).max(0.0);
                    let direction = wheel_forward_velocity[i].signum();
                    self.apply_force(
                        i,
                        -direction * vsc_force * wheel_forward_axes[i][0],
                        0.0,
                        -direction * vsc_force * wheel_forward_axes[i][1],
                    );
                }
            }
        }
        let brake_force =
            (brake_command + if self.controls.handbrake { 0.7 } else { 0.0 }) * mass * 9.81;
        if speed > 0.1 {
            let grounded_count = grounded.iter().filter(|is_grounded| **is_grounded).count();
            if grounded_count > 0 {
                for i in 0..4.min(count) {
                    if grounded[i] {
                        let direction = wheel_forward_velocity[i].signum();
                        self.apply_force(
                            i,
                            -direction * brake_force / grounded_count as f64
                                * wheel_forward_axes[i][0],
                            0.0,
                            -direction * brake_force / grounded_count as f64
                                * wheel_forward_axes[i][1],
                        );
                    }
                }
            }
        }

        let accel_g =
            ((forward_velocity - self.previous_forward_speed) / self.fixed_dt).abs() / 9.81;
        self.previous_forward_speed = forward_velocity;
        let drivetrain_forced_overrev = self.drivetrain.overrev_events.iter().any(|cause| {
            matches!(
                cause,
                drivetrain::OverrevCause::RapidDownshift
                    | drivetrain::OverrevCause::ClutchDump
                    | drivetrain::OverrevCause::WheelHop
                    | drivetrain::OverrevCause::EngineBraking
            )
        });
        self.drivetrain.overrev_events.clear();
        let engine_telemetry = update_engine_full_with_cause(
            &mut self.thermal,
            &self.cooling,
            &mut self.lubrication,
            &mut self.engine_damage,
            &mut self.engine_stress,
            self.drivetrain.engine.rpm,
            self.drivetrain.engine.redline_rpm,
            throttle,
            speed,
            accel_g,
            self.drivetrain.transmission.current_gear,
            self.time,
            self.fixed_dt,
            drivetrain_forced_overrev,
        );
        let fuel_state = if engine_running {
            self.fuel.update_fuel(
                self.drivetrain.engine.rpm,
                throttle,
                self.drivetrain.engine.redline_rpm,
                self.fixed_dt,
            )
        } else {
            suspension::FuelState {
                current_level: self.fuel.current_level,
                is_empty: self.fuel.current_level <= 0.0,
                fuel_mass: self.fuel.current_level * self.fuel.fuel_density,
            }
        };
        self.telemetry[T_COOLANT] = engine_telemetry.coolant_temp;
        self.telemetry[T_OIL_TEMP] = engine_telemetry.oil_temp;
        self.telemetry[T_OIL_PRESSURE] = engine_telemetry.oil_pressure;
        self.telemetry[T_TRANS_TEMP] = engine_telemetry.transmission_temp;
        self.telemetry[T_DERATE] = engine_telemetry.derate_factor;
        self.telemetry[T_FUEL] = fuel_state.current_level;
        self.telemetry[T_FUEL_PERCENT] = fuel_state.current_level / self.fuel.capacity * 100.0;
        self.telemetry[T_ENGINE_STAGE] = engine_telemetry.blowup_stage as u8 as f64;
        self.telemetry[T_LUGGING_STRESS] = engine_telemetry.lugging_stress;
        self.telemetry[T_OVERREV_STRESS] = engine_telemetry.overrev_stress;
        self.telemetry[T_SUSPENSION_LOAD] = suspension_load / contacts.max(1.0);
        self.telemetry[T_GRIP] = grip_sum / contacts.max(1.0);
        self.telemetry[T_TERRAIN_HEIGHT] = terrain_height / contacts.max(1.0);
        self.telemetry[T_CONTACTS] = contacts;
        self.telemetry[T_SLIP_RATIO] = average_slip_ratio / contacts.max(1.0);
        self.telemetry[T_RUT_DEPTH] = self
            .nodes
            .iter()
            .take(4)
            .map(|node| self.rut_depth_at(node.x, node.z))
            .sum::<f64>()
            / 4.0;
    }

    fn update_yaw_rate(&mut self) {
        if self.nodes.len() < 4 {
            self.yaw_rate = 0.0;
            return;
        }
        let front_x = (self.nodes[0].x + self.nodes[1].x) * 0.5;
        let front_z = (self.nodes[0].z + self.nodes[1].z) * 0.5;
        let rear_x = (self.nodes[2].x + self.nodes[3].x) * 0.5;
        let rear_z = (self.nodes[2].z + self.nodes[3].z) * 0.5;
        let yaw = (front_x - rear_x).atan2(-(front_z - rear_z));
        let mut delta = yaw - self.previous_yaw;
        while delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        while delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        self.yaw_rate = delta / self.fixed_dt;
        self.previous_yaw = yaw;
    }

    fn engine_derate(&self) -> f64 {
        if self.controls.engine_on {
            self.telemetry[T_DERATE].clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    fn apply_forces(&mut self) {
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            // Gravity belongs to each mass node. Suspension forces then travel
            // through the authored beams into the chassis instead of
            // teleporting upper-cage weight onto the wheel mounts.
            node.fy += node.mass * self.gravity;
        }
    }

    fn integrate_velocities(&mut self, dt: f64) {
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            node.vx += node.fx * node.inv_mass * dt;
            node.vy += node.fy * node.inv_mass * dt;
            node.vz += node.fz * node.inv_mass * dt;
            node.last_force = (node.fx * node.fx + node.fy * node.fy + node.fz * node.fz).sqrt();
            node.fx = 0.0;
            node.fy = 0.0;
            node.fz = 0.0;
            node.vx = finite_or_zero(node.vx).clamp(-200.0, 200.0);
            node.vy = finite_or_zero(node.vy).clamp(-200.0, 200.0);
            node.vz = finite_or_zero(node.vz).clamp(-200.0, 200.0);
            node.x = finite_or_zero(node.x + node.vx * dt);
            node.y = finite_or_zero(node.y + node.vy * dt);
            node.z = finite_or_zero(node.z + node.vz * dt);
        }
    }

    fn solve_xpbd_constraints(&mut self) {
        let dt2 = self.fixed_dt * self.fixed_dt;
        for beam in &mut self.beams {
            beam.lambda = 0.0;
        }
        for triangle in &mut self.triangles {
            triangle.lambda = 0.0;
        }
        for _ in 0..5 {
            for beam in &mut self.beams {
                if beam.broken || beam.node_a >= self.nodes.len() || beam.node_b >= self.nodes.len()
                {
                    continue;
                }
                let a = beam.node_a;
                let b = beam.node_b;
                let na = self.nodes[a];
                let nb = self.nodes[b];
                let dx = nb.x - na.x;
                let dy = nb.y - na.y;
                let dz = nb.z - na.z;
                let length = (dx * dx + dy * dy + dz * dz).sqrt();
                if length < 1e-8 {
                    continue;
                }
                let c = length - beam.length;
                let inv_sum = na.inv_mass + nb.inv_mass;
                if inv_sum <= 0.0 {
                    continue;
                }
                let compliance = 1.0 / beam.stiffness.max(1.0);
                let alpha = compliance / dt2;
                let delta_lambda = (-c - alpha * beam.lambda) / (inv_sum + alpha);
                beam.lambda += delta_lambda;
                let nx = dx / length;
                let ny = dy / length;
                let nz = dz / length;
                if !na.fixed {
                    self.nodes[a].x -= delta_lambda * na.inv_mass * nx;
                    self.nodes[a].y -= delta_lambda * na.inv_mass * ny;
                    self.nodes[a].z -= delta_lambda * na.inv_mass * nz;
                }
                if !nb.fixed {
                    self.nodes[b].x += delta_lambda * nb.inv_mass * nx;
                    self.nodes[b].y += delta_lambda * nb.inv_mass * ny;
                    self.nodes[b].z += delta_lambda * nb.inv_mass * nz;
                }
                let relative_velocity =
                    (nb.vx - na.vx) * nx + (nb.vy - na.vy) * ny + (nb.vz - na.vz) * nz;
                let damping = beam.damping * relative_velocity * self.fixed_dt;
                if !na.fixed {
                    self.nodes[a].vx += damping * na.inv_mass * nx;
                    self.nodes[a].vy += damping * na.inv_mass * ny;
                    self.nodes[a].vz += damping * na.inv_mass * nz;
                }
                if !nb.fixed {
                    self.nodes[b].vx -= damping * nb.inv_mass * nx;
                    self.nodes[b].vy -= damping * nb.inv_mass * ny;
                    self.nodes[b].vz -= damping * nb.inv_mass * nz;
                }
                let load = c.abs() * beam.stiffness + (na.last_force + nb.last_force) * 0.5;
                if load > beam.strength {
                    beam.broken = true;
                }
            }
            self.solve_triangle_constraints(dt2);
        }

        // XPBD projects positions after velocity integration. Feed only the
        // final projection correction back into velocities so a correction
        // that transfers a driven wheel's motion through the chassis is
        // reflected in telemetry, traction, and the next simulation step.
        let inv_dt = 1.0 / self.fixed_dt.max(1e-6);
        for (start, node) in self.constraint_start_positions.iter().zip(&mut self.nodes) {
            if node.fixed {
                continue;
            }
            node.vx += (node.x - start[0]) * inv_dt;
            node.vy += (node.y - start[1]) * inv_dt;
            node.vz += (node.z - start[2]) * inv_dt;
        }
    }

    // Area preservation is the soft-body equivalent of an angular/bend
    // constraint for the mesh triangles. It resists shearing while still
    // allowing beams to break and the body to crumple.
    fn solve_triangle_constraints(&mut self, dt2: f64) {
        for tri in &mut self.triangles {
            if tri.a >= self.nodes.len() || tri.b >= self.nodes.len() || tri.c >= self.nodes.len() {
                continue;
            }
            let center = {
                let a = self.nodes[tri.a];
                let b = self.nodes[tri.b];
                let c = self.nodes[tri.c];
                [
                    (a.x + b.x + c.x) / 3.0,
                    (a.y + b.y + c.y) / 3.0,
                    (a.z + b.z + c.z) / 3.0,
                ]
            };
            let area = triangle_area(&self.nodes[tri.a], &self.nodes[tri.b], &self.nodes[tri.c]);
            let error = area - tri.rest_area;
            let alpha = 0.05 / dt2;
            let delta = (-error - alpha * tri.lambda) / (1.0 + alpha);
            tri.lambda += delta;
            for index in [tri.a, tri.b, tri.c] {
                if !self.nodes[index].fixed {
                    self.nodes[index].x += (self.nodes[index].x - center[0]) * delta * 0.02;
                    self.nodes[index].y += (self.nodes[index].y - center[1]) * delta * 0.02;
                    self.nodes[index].z += (self.nodes[index].z - center[2]) * delta * 0.02;
                }
            }
        }
    }

    fn collide_with_terrain(&mut self) {
        let profile = self.terrain_profile;
        self.impact_severity *= 0.90_f64.powf(self.fixed_dt * 60.0);
        let (center_x, center_z, total_mass) = {
            let dynamic_count = self.nodes.iter().filter(|node| !node.fixed).count().max(1) as f64;
            (
                self.nodes
                    .iter()
                    .filter(|node| !node.fixed)
                    .map(|node| node.x)
                    .sum::<f64>()
                    / dynamic_count,
                self.nodes
                    .iter()
                    .filter(|node| !node.fixed)
                    .map(|node| node.z)
                    .sum::<f64>()
                    / dynamic_count,
                self.nodes
                    .iter()
                    .map(|node| node.mass)
                    .sum::<f64>()
                    .max(1.0),
            )
        };
        for index in 0..self.nodes.len() {
            let (x, z, collision) = {
                let node = &self.nodes[index];
                (node.x, node.z, node.collision)
            };
            if !collision {
                continue;
            }
            let height = terrain_height_for_profile(profile, x, z) + self.rut_depth_at(x, z);
            let normal = self.terrain_normal(x, z);
            let node_snapshot = self.nodes[index];
            if node_snapshot.y < height {
                let normal_velocity = node_snapshot.vx * normal[0]
                    + node_snapshot.vy * normal[1]
                    + node_snapshot.vz * normal[2];
                if normal_velocity < 0.0 {
                    let impact_impulse = -normal_velocity * node_snapshot.mass;
                    if impact_impulse > total_mass * 0.25 {
                        let severity = (impact_impulse / (total_mass * 8.0)).clamp(0.0, 1.0);
                        let zone = classify_damage_zone(
                            node_snapshot.x,
                            node_snapshot.y,
                            node_snapshot.z,
                            center_x,
                            center_z,
                        );
                        self.impact_severity = self.impact_severity.max(severity);
                        self.body_damage = (self.body_damage + severity * 0.03).clamp(0.0, 1.0);
                        self.damage_zone = zone;
                        self.impact_history.push(ImpactEvent {
                            timestamp: self.time,
                            severity,
                            zone,
                        });
                        if self.impact_history.len() > 32 {
                            self.impact_history.remove(0);
                        }
                    }
                }
                let node = &mut self.nodes[index];
                node.y = height;
                if normal_velocity < 0.0 {
                    let restitution = 0.15;
                    node.vx -= normal_velocity * (1.0 + restitution) * normal[0];
                    node.vy -= normal_velocity * (1.0 + restitution) * normal[1];
                    node.vz -= normal_velocity * (1.0 + restitution) * normal[2];
                }
                let friction = if profile == 2 { 0.82 } else { 0.94 };
                node.vx *= friction;
                node.vz *= friction;
            }
        }
    }

    fn apply_drag(&mut self) {
        let total_mass = self
            .nodes
            .iter()
            .filter(|node| !node.fixed)
            .map(|node| node.mass)
            .sum::<f64>()
            .max(1.0);
        let average_velocity =
            self.nodes
                .iter()
                .filter(|node| !node.fixed)
                .fold([0.0; 3], |mut average, node| {
                    average[0] += node.vx * node.mass / total_mass;
                    average[1] += node.vy * node.mass / total_mass;
                    average[2] += node.vz * node.mass / total_mass;
                    average
                });
        let speed = (average_velocity[0] * average_velocity[0]
            + average_velocity[1] * average_velocity[1]
            + average_velocity[2] * average_velocity[2])
            .sqrt();
        let total_drag =
            0.5 * self.air_density * self.drag_coefficient * self.frontal_area * speed * speed;
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            let node_speed = (node.vx * node.vx + node.vy * node.vy + node.vz * node.vz).sqrt();
            if node_speed > 1e-6 {
                // Aerodynamic drag is defined by frontal area, not vehicle
                // mass. Distribute the vehicle drag by node mass so the
                // deformable cage receives one bounded body-level force.
                let drag = total_drag * node.mass / total_mass;
                let factor =
                    (1.0 - drag * self.fixed_dt / (node.mass * node_speed + 1e-6)).max(0.0);
                node.vx *= factor;
                node.vy *= factor;
                node.vz *= factor;
            }
        }
    }

    fn terrain_height(&self, x: f64, z: f64) -> f64 {
        terrain_height_for_profile(self.terrain_profile, x, z) + self.rut_depth_at(x, z)
    }

    fn rut_depth_at(&self, x: f64, z: f64) -> f64 {
        let cell = terrain_cell(x, z);
        self.terrain_ruts[cell.1 * TERRAIN_GRID_SIZE + cell.0]
    }

    fn terrain_normal(&self, x: f64, z: f64) -> [f64; 3] {
        let e = 0.25;
        let h_l = self.terrain_height(x - e, z);
        let h_r = self.terrain_height(x + e, z);
        let h_d = self.terrain_height(x, z - e);
        let h_u = self.terrain_height(x, z + e);
        let normal = [h_l - h_r, 2.0 * e, h_d - h_u];
        let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])
            .sqrt()
            .max(1e-6);
        [normal[0] / len, normal[1] / len, normal[2] / len]
    }

    fn deposit_rut(&mut self, x: f64, z: f64, load: f64, slip_ratio: f64, deformability: f64) {
        if deformability <= 0.0 || load <= 0.0 || !slip_ratio.is_finite() {
            return;
        }
        let slip = slip_ratio.abs();
        if slip < 0.03 {
            return;
        }
        let (cx, cz) = terrain_cell(x, z);
        let amount = (slip * load * deformability * self.fixed_dt * 0.000001).clamp(0.0, 0.004);
        for dz in cz.saturating_sub(1)..=(cz + 1).min(TERRAIN_GRID_SIZE - 1) {
            for dx in cx.saturating_sub(1)..=(cx + 1).min(TERRAIN_GRID_SIZE - 1) {
                let distance =
                    ((dx as isize - cx as isize).abs() + (dz as isize - cz as isize).abs()) as f64;
                let weight = if distance == 0.0 { 1.0 } else { 0.35 };
                let index = dz * TERRAIN_GRID_SIZE + dx;
                self.terrain_ruts[index] = (self.terrain_ruts[index] + amount * weight).min(0.25);
            }
        }
    }

    fn terrain_contact(&self, x: f64, z: f64) -> TerrainContact {
        let presets = surface_presets();
        let mut surface = match self.terrain_profile {
            1 => presets[0].clone(),
            2 => presets[3].clone(),
            _ => presets[0].clone(),
        };
        if self.terrain_profile == 1 {
            surface.roughness = 0.45;
        }
        let normal = self.terrain_normal(x, z);
        TerrainContact {
            surface,
            normal,
            slope_angle: normal[1].acos(),
            moisture: if self.terrain_profile == 2 { 0.25 } else { 0.0 },
            compactness: if self.terrain_profile == 2 { 0.7 } else { 1.0 },
            rut_depth: 0.0,
        }
    }

    fn update_telemetry(&mut self) {
        let count = self.nodes.iter().filter(|n| !n.fixed).count().max(1) as f64;
        let vx = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.vx)
            .sum::<f64>()
            / count;
        let vz = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| n.vz)
            .sum::<f64>()
            / count;
        let speed = (vx * vx + vz * vz).sqrt();
        self.telemetry[T_SPEED_MPS] = speed;
        self.telemetry[T_SPEED_KMH] = speed * 3.6;
        self.telemetry[T_RPM] = if self.controls.engine_on {
            self.drivetrain.get_rpm()
        } else {
            0.0
        };
        self.telemetry[T_GEAR] = self.drivetrain.get_gear() as f64;
        self.telemetry[T_THROTTLE] = self.controls.throttle;
        self.telemetry[T_BRAKE] = self.controls.brake;
        self.telemetry[T_STEERING] = self.steering_angle;
        self.telemetry[T_CLUTCH] = self.drivetrain.get_clutch();
        self.telemetry[T_WHEEL_SPEED] = self.drivetrain.wheel_speed;
        self.telemetry[T_DRIVE_TORQUE] = self.drivetrain.get_drive_torque();
        self.telemetry[T_ENGINE_RUNNING] = if self.controls.engine_on
            && self.fuel.current_level > 0.0
            && !self.engine_damage.is_seized
        {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_SHIFT_PHASE] = match self.drivetrain.shift_phase {
            drivetrain::ShiftPhase::Idle => 0.0,
            drivetrain::ShiftPhase::Disengaging => 1.0,
            drivetrain::ShiftPhase::Neutral => 2.0,
            drivetrain::ShiftPhase::Engaging => 3.0,
        };
        self.telemetry[T_ABS_ACTIVE] = if self.safety.abs.is_active { 1.0 } else { 0.0 };
        self.telemetry[T_TC_ACTIVE] = if self.safety.traction_control.is_active {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_VSC_ACTIVE] = if self.safety.vsc_esc.is_active {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_TCM_STATE] = self.tcm.state as u8 as f64;
        self.telemetry[T_TCM_LINE_PRESSURE] = self.tcm.line_pressure;
        self.telemetry[T_TCM_LOCKUP] = if self.tcm.converter_lockup { 1.0 } else { 0.0 };
        self.telemetry[T_REQUESTED_GEAR] = self.drivetrain.pending_gear as f64;
        self.telemetry[T_INPUT_SHAFT_RPM] = self.drivetrain.engine.rpm;
        self.telemetry[T_OUTPUT_SHAFT_RPM] =
            self.drivetrain.wheel_speed.abs() * 60.0 / (2.0 * std::f64::consts::PI);
        self.telemetry[T_CONVERTER_COUPLING] = self.drivetrain.converter_coupling;
        self.telemetry[T_BROKEN_BEAMS] = self.beams.iter().filter(|b| b.broken).count() as f64;
        let structural_damage = self.telemetry[T_BROKEN_BEAMS] / self.beams.len().max(1) as f64;
        let engine_damage = self
            .engine_damage
            .wear_level
            .max(self.engine_damage.bearing_damage);
        self.telemetry[T_DAMAGE] = structural_damage
            .max(engine_damage)
            .max(self.body_damage)
            .clamp(0.0, 1.0);
        self.telemetry[T_ENGINE_WARNING] = if self.telemetry[T_COOLANT] > 105.0
            || self.telemetry[T_OIL_PRESSURE] < 80.0
            || self.telemetry[T_ENGINE_STAGE] > 0.0
        {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_ADAS_FCW] = if self.safety.adas.forward_collision_warning {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_ADAS_AEB] = if self.safety.adas.aeb_active {
            1.0
        } else {
            0.0
        };
        self.telemetry[T_ADAS_CONFIDENCE] = self.safety.adas.front_target.confidence;
        self.telemetry[T_TIRE_WEAR] = self.tires.iter().map(|t| t.wear).sum::<f64>() / 4.0;
        self.telemetry[T_TIRE_TEMP] = self.tires.iter().map(|t| t.temperature).sum::<f64>() / 4.0;
        for i in 0..4 {
            self.telemetry[T_WHEEL_SPEED_FL + i] =
                self.wheels[i].angular_speed.abs() * self.suspension.wheels[i].tire_radius * 3.6;
            self.telemetry[T_SUSPENSION_COMPRESSION_FL + i] = self.wheels[i].compression * 100.0;
            self.telemetry[T_SUSPENSION_LOAD_FL + i] = self.wheels[i].load;
            self.telemetry[T_TIRE_TEMP_FL + i] = self.tires[i].temperature;
            self.telemetry[T_TIRE_WEAR_FL + i] = self.tires[i].wear;
        }
        let deformation_depth = self
            .beams
            .iter()
            .map(|beam| (beam.length - beam.initial_length).abs())
            .fold(0.0, f64::max);
        self.telemetry[T_IMPACT_SEVERITY] = self.impact_severity;
        self.telemetry[T_DEFORMATION_DEPTH] = deformation_depth;
        self.telemetry[T_BROKEN_PARTS] = self.telemetry[T_BROKEN_BEAMS];
        self.telemetry[T_DAMAGE_ZONE] = self.damage_zone as f64;
        self.telemetry[T_SIM_TIME] = self.time;
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

fn terrain_height_for_profile(profile: u8, x: f64, z: f64) -> f64 {
    match profile {
        // Keep these coefficients in lockstep with useTerrain.ts. The
        // renderer and worker must agree on the contact height or tires will
        // visibly float/sink as soon as the vehicle leaves the spawn point.
        1 => {
            (x * 0.12).sin() * 0.28 * 7.0 * 0.08
                + (z * 0.17).sin() * 0.18 * 7.0 * 0.08
                + ((x + z) * 0.045).sin() * 0.14 * 7.0 * 0.08
        }
        2 => {
            ((x * 0.035).sin() * 0.55
                + (z * 0.027).cos() * 0.4
                + ((x - z) * 0.09).sin() * 0.22
                + ((x + z) * 0.065).cos() * 0.16)
                * 13.0
                * 0.12
        }
        _ => 0.0,
    }
}

fn terrain_cell(x: f64, z: f64) -> (usize, usize) {
    let gx = ((x / TERRAIN_EXTENT + 0.5) * TERRAIN_GRID_SIZE as f64).floor();
    let gz = ((z / TERRAIN_EXTENT + 0.5) * TERRAIN_GRID_SIZE as f64).floor();
    (
        gx.clamp(0.0, (TERRAIN_GRID_SIZE - 1) as f64) as usize,
        gz.clamp(0.0, (TERRAIN_GRID_SIZE - 1) as f64) as usize,
    )
}

/// Collision zone identifiers: 0 unknown, 1 front, 2 rear, 3 side,
/// 4 roof, 5 underbody. The zone remains in telemetry after the impulse
/// decays so damage reports retain causal context.
fn classify_damage_zone(x: f64, y: f64, z: f64, center_x: f64, center_z: f64) -> u8 {
    let dx = (x - center_x).abs();
    let dz = z - center_z;
    if y > 1.2 {
        4
    } else if y < 0.15 {
        5
    } else if dz < -0.35 {
        1
    } else if dz > 0.35 {
        2
    } else if dx > 0.55 {
        3
    } else {
        0
    }
}
fn sane_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}
fn value_or(values: &[f64], index: usize, fallback: f64) -> f64 {
    values
        .get(index)
        .copied()
        .filter(|v| v.is_finite())
        .unwrap_or(fallback)
}
fn value_or_u8(values: &[u8], index: usize, fallback: u8) -> u8 {
    values.get(index).copied().unwrap_or(fallback)
}
fn distance(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> f64 {
    ((bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2)).sqrt()
}
fn triangle_area(a: &Node, b: &Node, c: &Node) -> f64 {
    let ab = [b.x - a.x, b.y - a.y, b.z - a.z];
    let ac = [c.x - a.x, c.y - a.y, c.z - a.z];
    let cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn car_world() -> PhysicsWorld {
        let mut w = PhysicsWorld::new();
        for (i, (x, y, z)) in [
            (-0.7, 0.5, -0.6),
            (0.7, 0.5, -0.6),
            (-0.7, 0.5, 0.6),
            (0.7, 0.5, 0.6),
            (-0.7, 1.0, -0.6),
            (0.7, 1.0, -0.6),
            (-0.7, 1.0, 0.6),
            (0.7, 1.0, 0.6),
        ]
        .iter()
        .enumerate()
        {
            w.add_node(i, *x, *y, *z, 50.0, false);
        }
        for (i, (a, b)) in [
            (0, 1),
            (2, 3),
            (4, 5),
            (6, 7),
            (0, 2),
            (1, 3),
            (4, 6),
            (5, 7),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ]
        .iter()
        .enumerate()
        {
            w.add_beam(i, *a, *b, 10000.0, 0.5, 100000.0);
        }
        w
    }

    #[test]
    fn node_creation_and_gravity() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 1.0, 10.0, 3.0, 5.0, false);
        w.step(1.0 / 60.0);
        assert!(w.nodes[0].y < 10.0 && w.nodes[0].vy < 0.0);
    }

    #[test]
    fn fixed_timestep_is_bounded() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 10.0, 0.0, 1.0, false);
        w.step(1.0);
        assert!(w.time <= 8.0 / 60.0 + 1e-9);
    }

    #[test]
    fn xpbd_beam_rest_length_is_stable() {
        let mut w = car_world();
        let rest = w.beams[0].length;
        for _ in 0..120 {
            w.step(1.0 / 60.0);
        }
        assert!(
            (distance(
                w.nodes[0].x,
                w.nodes[0].y,
                w.nodes[0].z,
                w.nodes[1].x,
                w.nodes[1].y,
                w.nodes[1].z
            ) - rest)
                .abs()
                < 0.2
        );
    }

    #[test]
    fn controls_and_terrain_reach_telemetry() {
        let mut w = car_world();
        w.set_terrain_profile(2);
        let lift = (0..4)
            .map(|index| {
                let node = w.nodes[index];
                let wheel = &w.suspension.wheels[index];
                w.terrain_height(node.x, node.z) + wheel.rest_length + wheel.tire_radius - node.y
            })
            .fold(0.0, f64::max);
        for node in &mut w.nodes {
            node.y += lift;
        }
        w.set_controls(0.2, 1.0, 0.0, 0.0, false, false, false, true);
        for _ in 0..30 {
            w.step(1.0 / 60.0);
        }
        assert!(w.telemetry[T_RPM] > 0.0);
        assert!(w.telemetry[T_CONTACTS] > 0.0);
        assert!(w.telemetry[T_GRIP] >= 0.0);
    }

    #[test]
    fn engine_off_produces_no_drive_torque() {
        let mut w = car_world();
        let fuel_before = w.fuel.current_level;
        w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, false);
        w.step(1.0 / 60.0);
        assert_eq!(w.telemetry[T_RPM], 0.0);
        assert_eq!(w.telemetry[T_DRIVE_TORQUE], 0.0);
        assert_eq!(
            w.fuel.current_level, fuel_before,
            "engine-off vehicle must not burn fuel"
        );
    }

    #[test]
    fn positive_drive_torque_moves_vehicle_toward_negative_z() {
        let mut w = car_world();
        let initial_rear_z = (w.nodes[2].z + w.nodes[3].z) * 0.5;
        w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, true);
        for _ in 0..120 {
            w.step(1.0 / 60.0);
        }
        let final_rear_z = (w.nodes[2].z + w.nodes[3].z) * 0.5;
        assert!(
            final_rear_z < initial_rear_z - 0.05,
            "forward drive must move along authored -Z axis: {initial_rear_z} -> {final_rear_z}"
        );
    }

    #[test]
    fn airborne_driven_wheels_do_not_accelerate_chassis() {
        let mut w = car_world();
        for node in &mut w.nodes {
            node.y += 4.0;
        }
        let initial_z = w.nodes.iter().map(|node| node.z).sum::<f64>();
        w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, true);
        w.step(1.0 / 60.0);
        let final_z = w.nodes.iter().map(|node| node.z).sum::<f64>();
        assert!(
            (final_z - initial_z).abs() < 1e-6,
            "airborne wheels must not create ground drive force"
        );
    }

    #[test]
    fn positive_steering_turns_vehicle_toward_positive_x() {
        let mut w = car_world();
        w.set_controls(1.0, 0.7, 0.0, 0.0, false, false, false, true);
        for _ in 0..240 {
            w.step(1.0 / 60.0);
        }
        let front_x = (w.nodes[0].x + w.nodes[1].x) * 0.5;
        let rear_x = (w.nodes[2].x + w.nodes[3].x) * 0.5;
        assert!(
            front_x - rear_x > 0.02,
            "positive steering should rotate the vehicle toward +X: front={front_x}, rear={rear_x}"
        );
    }

    #[test]
    fn configured_drive_reaches_speed_and_preserves_wheel_mounts() {
        let mut w = car_world();
        let initial_wheel_z = w.nodes.iter().take(4).map(|node| node.z).sum::<f64>() / 4.0;
        let initial_body_z =
            w.nodes.iter().skip(4).map(|node| node.z).sum::<f64>() / (w.nodes.len() - 4) as f64;
        for node in &mut w.nodes {
            node.y += 0.18;
        }
        w.configure_runtime(
            800.0,
            7000.0,
            7200.0,
            0.8,
            0.3,
            &[0.0, 1000.0, 3000.0, 7000.0],
            &[100.0, 150.0, 250.0, 160.0],
            &[3.5, 2.1, 1.4, 1.0, 0.7],
            3.7,
            -3.2,
            1,
            0.15,
            0,
            0.5,
            &[30_000.0; 4],
            &[4_000.0; 4],
            &[2_500.0; 4],
            &[0.35; 4],
            &[0.2; 4],
            &[0.33; 4],
            &[1; 4],
            &[32.0; 4],
            60.0,
            0.01,
            0.0005,
            false,
            false,
            false,
            false,
            false,
        );
        w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, true);
        for step in 0..1200 {
            w.step(1.0 / 60.0);
            if step % 60 == 0 {
                eprintln!(
                    "auto debug t={} gear={} rpm={} speed={}",
                    step,
                    w.telemetry[T_GEAR],
                    w.telemetry[T_RPM],
                    w.telemetry[T_SPEED_MPS]
                );
            }
        }
        assert!(w.telemetry[T_SPEED_MPS] > 0.5);
        assert!(
            w.telemetry[T_GEAR] > 1.0,
            "automatic drivetrain should upshift while accelerating"
        );
        assert!(w.nodes.iter().take(4).all(|node| node.y > 0.2));
        let wheel_z = w.nodes.iter().take(4).map(|node| node.z).sum::<f64>() / 4.0;
        let body_z =
            w.nodes.iter().skip(4).map(|node| node.z).sum::<f64>() / (w.nodes.len() - 4) as f64;
        assert!(
            ((body_z - wheel_z) - (initial_body_z - initial_wheel_z)).abs() < 0.35,
            "the upper body cage must follow the wheel mounts under drive"
        );
        assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
    }

    #[test]
    fn deformable_terrain_creates_bounded_ruts_and_resets_with_profile() {
        let mut w = PhysicsWorld::new();
        w.set_terrain_profile(2);
        w.deposit_rut(0.0, 0.0, 20_000.0, 1.0, 0.9);
        assert!(w.rut_depth_at(0.0, 0.0) > 0.0);
        assert!(w.rut_depth_at(0.0, 0.0) <= 0.25);
        w.set_terrain_profile(0);
        assert_eq!(w.rut_depth_at(0.0, 0.0), 0.0);
    }
}
