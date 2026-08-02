use crate::drivetrain::{self, TransmissionMode};
use crate::engine::update_engine_full_with_cause;
use crate::math::{
    classify_damage_zone, finite_or_zero, terrain_height_for_profile, triangle_area,
};
use crate::suspension::{self, raycast_wheel};
use crate::terrain_contact::calculate_traction;
use crate::tires::update_tire;
use crate::types::*;

const BODY_NODE_CLEARANCE: f64 = 0.2;

impl PhysicsWorld {
    pub(crate) fn step_fixed(&mut self) {
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

    pub(crate) fn apply_vehicle_forces(&mut self) {
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
            self.controls.throttle * tc_modifier * (1.0 - self.tcm.torque_reduction_request)
        } else {
            0.0
        };
        if engine_running {
            if self.drivetrain.transmission.mode == TransmissionMode::Automatic {
                let coupling = if self.tcm.converter_lockup {
                    1.0
                } else {
                    (0.45 + self.tcm.line_pressure * 0.30).clamp(0.3, 0.85)
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
                if let Some(target_gear) = self.tcm.update_with_brake(
                    self.drivetrain.engine.rpm,
                    speed * 3.6,
                    throttle,
                    self.controls.brake,
                    0.0,
                    self.telemetry[T_TRANS_TEMP],
                    current_gear,
                    self.drivetrain.transmission.gear_ratios.len() as i32,
                    self.fixed_dt,
                ) {
                    self.drivetrain
                        .set_shift_duration_multiplier(self.tcm.shift_duration_multiplier());
                    if target_gear > current_gear {
                        self.drivetrain.request_shift_up();
                    } else if target_gear < current_gear
                        && !self.drivetrain.check_downshift_overrev(target_gear).1
                    {
                        self.drivetrain.request_shift_down();
                    }
                } else if self.tcm.limp_mode {
                    self.drivetrain
                        .set_shift_duration_multiplier(self.tcm.shift_duration_multiplier());
                    let fail_safe_gear = self
                        .tcm
                        .fail_safe_gear(self.drivetrain.transmission.gear_ratios.len() as i32);
                    if fail_safe_gear > current_gear {
                        self.drivetrain.request_shift_up();
                    } else if fail_safe_gear < current_gear
                        && !self.drivetrain.check_downshift_overrev(fail_safe_gear).1
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
                // Keep the low-speed denominators physical. Using 1 m/s as
                // a blanket epsilon made a steered wheel look straight until
                // the car was already moving quickly, which read as ignored
                // steering and also hid wheel spin during launch.
                let slip_ratio = (driven_wheel_velocity - ground_forward_velocity)
                    / ground_forward_velocity.abs().max(0.25);
                let slip_angle =
                    ground_lateral_velocity.atan2(ground_forward_velocity.abs().max(0.1));
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
        let mut transmitted_torque = 0.0;
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
                transmitted_torque += force_forward * wheel_radius;
                self.apply_force(
                    i,
                    force_forward * wheel_forward_axes[i][0],
                    0.0,
                    force_forward * wheel_forward_axes[i][1],
                );
            }
        }
        self.drivetrain
            .apply_wheel_reaction_torque(transmitted_torque, self.fixed_dt, mass);
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

    pub(crate) fn update_yaw_rate(&mut self) {
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

    pub(crate) fn engine_derate(&self) -> f64 {
        if self.controls.engine_on {
            self.telemetry[T_DERATE].clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    pub(crate) fn apply_forces(&mut self) {
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

    pub(crate) fn integrate_velocities(&mut self, dt: f64) {
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

    pub(crate) fn solve_xpbd_constraints(&mut self) {
        let dt2 = self.fixed_dt * self.fixed_dt;
        for beam in &mut self.beams {
            beam.lambda = 0.0;
        }
        for triangle in &mut self.triangles {
            triangle.lambda = 0.0;
        }
        for _ in 0..10 {
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
                // Beam strength is a tensile yield limit. Endpoint force is
                // not a beam load: using each node's total force here makes
                // ordinary suspension and tire forces break axle/vertical
                // members even when they are still at (or below) rest length.
                // Estimate the axial load from extension only; compression
                // buckling is a separate damage model and must not make a
                // healthy chassis disappear during launch.
                let tensile_load = c.max(0.0) * beam.stiffness;
                if tensile_load > beam.strength {
                    beam.broken = true;
                }
            }
            self.solve_triangle_constraints(dt2);
            self.solve_body_attachment_constraints();
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

    /// Keep the authored upper body cage attached to the suspension-mount
    /// frame. A distance-only beam graph can preserve every beam length while
    /// folding the roof and body layer into the road (a valid linkage, but not
    /// a usable chassis). The authored vertical offsets are the suspension
    /// attachment geometry: they are allowed to move upward with a bump, but
    /// cannot collapse below the corresponding wheel mount under normal
    /// gravity or drive load. Collision damage still acts through broken
    /// beams and the existing terrain deformation path.
    fn solve_body_attachment_constraints(&mut self) {
        let node_count = self.nodes.len().min(self.rest_positions.len());
        if node_count < 8 {
            return;
        }
        let upper_count = node_count.min(12);
        for index in 4..upper_count {
            // The standard eight-node car maps body nodes 4..7 directly to
            // mounts 0..3. For taller/irregular layouts, attach each further
            // upper node to the nearest authored body node in the x/z plane;
            // this also keeps the two-node ATV top layer connected without
            // assuming it has the 12-node car ordering.
            let reference = if index < 8 {
                index - 4
            } else {
                let mut nearest = 4;
                let mut nearest_distance = f64::INFINITY;
                for candidate in 4..node_count.min(8) {
                    let dx = self.rest_positions[index][0] - self.rest_positions[candidate][0];
                    let dz = self.rest_positions[index][2] - self.rest_positions[candidate][2];
                    let candidate_distance = dx * dx + dz * dz;
                    if candidate_distance < nearest_distance {
                        nearest = candidate;
                        nearest_distance = candidate_distance;
                    }
                }
                nearest
            };
            if reference >= self.nodes.len() {
                continue;
            }
            let authored_offset = self.rest_positions[index][1] - self.rest_positions[reference][1];
            if !authored_offset.is_finite() || authored_offset <= 0.0 {
                continue;
            }
            let direct_beam_broken = self.beams.iter().any(|beam| {
                beam.broken
                    && ((beam.node_a == reference && beam.node_b == index)
                        || (beam.node_a == index && beam.node_b == reference))
            });
            if direct_beam_broken {
                continue;
            }
            let upper = self.nodes[index];
            let mount = self.nodes[reference];
            let constraint_error = upper.y - mount.y - authored_offset;
            let inv_mass_sum = upper.inv_mass + mount.inv_mass;
            if inv_mass_sum <= 0.0 {
                continue;
            }
            // Project both endpoints. Moving only the upper node prevents the
            // body mass from ever reaching the suspension mounts, producing
            // unrealistically low tire load and a powered car that spins its
            // wheels instead of accelerating. Equal-and-opposite projection
            // lets the mount settle into the spring and transfers the upper
            // cage's weight through the chassis connection.
            let correction = -constraint_error / inv_mass_sum;
            if !mount.fixed {
                self.nodes[reference].y -= correction * mount.inv_mass;
            }
            if !upper.fixed {
                self.nodes[index].y += correction * upper.inv_mass;
            }
        }
    }

    // Area preservation is the soft-body equivalent of an angular/bend
    // constraint for the mesh triangles. It resists shearing while still
    // allowing beams to break and the body to crumple.
    pub(crate) fn solve_triangle_constraints(&mut self, dt2: f64) {
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

    pub(crate) fn collide_with_terrain(&mut self) {
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
            let terrain_height =
                terrain_height_for_profile(profile, x, z) + self.rut_depth_at(x, z);
            // Upper-cage nodes are point samples of a body shell, not tire
            // contact points. Give them a small underbody clearance so a
            // soft-body solver cannot legally place the whole chassis center
            // on the terrain while the suspension mounts remain supported.
            let height = terrain_height + if index >= 4 { BODY_NODE_CLEARANCE } else { 0.0 };
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
                // The clearance plane is an underbody safety proxy, not a
                // tire contact patch. Applying ground friction whenever an
                // upper node is clamped to that proxy turns the chassis into
                // a brake and leaves the powered car barely moving. Only a
                // true penetration of the terrain surface receives impact
                // friction; rolling resistance is handled per wheel.
                if node_snapshot.y < terrain_height {
                    let friction = if profile == 2 { 0.82 } else { 0.94 };
                    node.vx *= friction;
                    node.vz *= friction;
                }
            }
        }
    }

    pub(crate) fn apply_drag(&mut self) {
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
}
