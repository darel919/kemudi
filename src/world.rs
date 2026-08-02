use wasm_bindgen::prelude::*;

use crate::drivetrain::{
    self, DiffMode, Drivetrain, TCMFaultKind, TransmissionControlModule, TransmissionMode,
};
use crate::engine::{
    CoolingSystem, EngineDamage, EngineStressAccumulators, EngineThermal, LubricationSystem,
};
use crate::math::{distance, finite_or_zero, sane_or, triangle_area, value_or, value_or_u8};
use crate::safety::SafetySystemState;
use crate::suspension::{self, SteeringConfig, SuspensionConfig, WheelState};
use crate::tires::{TireCompound, TireState, TireThermalParams};
use crate::types::{
    Beam, Controls, Node, PhysicsWorld, TriangleConstraint, TELEMETRY_LEN, TERRAIN_GRID_SIZE,
    T_DERATE,
};

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
            rest_positions: Vec::new(),
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
        self.rest_positions.push([safe_x, safe_y, safe_z]);
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
        anti_roll_bar_stiffness: f64,
        bump_stop_rate: f64,
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
        // Runtime configuration is also the reset boundary used by the
        // worker. Never inherit a gear, shaft speed, shift phase, or limp
        // state from a previous vehicle/configuration. Every transmission
        // launches in first gear; reverse is selected explicitly by the
        // driver through a manual downshift at rest.
        transmission.current_gear = if transmission.gear_ratios.is_empty() {
            0
        } else {
            1
        };
        transmission.clutch_engagement = 1.0;
        self.drivetrain.engine.rpm = self.drivetrain.engine.idle_rpm;
        self.drivetrain.wheel_speed = 0.0;
        self.drivetrain.vehicle_speed = 0.0;
        self.drivetrain.shift_phase = drivetrain::ShiftPhase::Idle;
        self.drivetrain.shift_timer = 0.0;
        self.drivetrain.pending_gear = transmission.current_gear;
        self.drivetrain.time = 0.0;
        self.drivetrain.is_stalled = false;
        self.drivetrain.last_drive_torque = 0.0;
        self.drivetrain.converter_coupling = 1.0;
        self.drivetrain.converter_torque_multiplier = 1.0;
        self.tcm.enabled = transmission.mode == TransmissionMode::Automatic;
        self.tcm.state = drivetrain::TCMState::Normal;
        self.tcm.limp_mode = false;
        self.tcm.pending_shift = None;
        self.tcm.converter_lockup = false;
        self.tcm.time_since_shift = self.tcm.min_shift_interval;
        self.tcm.clear_faults();
        self.drivetrain.shift_duration_multiplier = 1.0;
        self.drivetrain.shift_duration = sane_or(shift_delay, 0.15).clamp(0.05, 1.0);
        self.drivetrain.auto_shift.shift_delay = self.drivetrain.shift_duration;
        self.drivetrain.differential.mode = match differential_mode {
            1 => DiffMode::Locked,
            2 => DiffMode::LimitedSlip,
            _ => DiffMode::Open,
        };
        self.drivetrain.differential.bias = sane_or(differential_bias, 0.5).clamp(0.0, 1.0);
        self.suspension.anti_roll_bar_stiffness = sane_or(
            anti_roll_bar_stiffness,
            self.suspension.anti_roll_bar_stiffness,
        )
        .max(0.0);
        self.suspension.bump_stop_rate =
            sane_or(bump_stop_rate, self.suspension.bump_stop_rate).max(0.0);

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

    /// Inject or clear a deterministic TCM fault. Fault IDs are stable across
    /// the worker boundary; see `TCMFaultKind` for the mapping.
    pub fn set_tcm_fault(&mut self, fault_id: u8, active: bool) {
        if let Some(kind) = TCMFaultKind::from_u8(fault_id) {
            self.tcm.set_fault(kind, active);
        }
    }

    pub fn set_tcm_fault_intermittent(&mut self, fault_id: u8, intermittent: bool) {
        if let Some(kind) = TCMFaultKind::from_u8(fault_id) {
            self.tcm.set_fault_intermittent(kind, intermittent);
        }
    }

    pub fn set_tcm_fault_seed(&mut self, seed: u64) {
        self.tcm.set_fault_seed(seed);
    }

    pub fn clear_tcm_faults(&mut self) {
        self.tcm.clear_faults();
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

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}
