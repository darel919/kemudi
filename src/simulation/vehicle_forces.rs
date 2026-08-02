//! Vehicle controls, drivetrain, suspension, tire contact, safety, and thermal forces.

use crate::drivetrain::{self, TransmissionMode};
use crate::engine::update_engine_full_with_cause;
use crate::suspension::{self, raycast_wheel, suspension_length_velocity};
use crate::terrain_contact::calculate_traction;
use crate::tires::update_tire;
use crate::types::*;

impl PhysicsWorld {
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
        let driven_contact = (2..4.min(count)).any(|index| {
            let node = self.nodes[index];
            let height = self.terrain_height(node.x, node.z);
            !raycast_wheel(
                &self.suspension.wheels[index],
                [node.x, node.y, node.z],
                0.0,
                height,
            )
            .2
        });
        if !engine_running || (self.controls.throttle <= f64::EPSILON && driven_contact) {
            // An unpowered or zero-throttle driven wheel free-rolls with the
            // chassis. Static tire contact is the kinematic authority here:
            // retaining an independently integrated shaft speed lets a tiny
            // settling velocity repeatedly flip a constant engine-brake
            // torque and inject energy at every fixed step.
            self.drivetrain.wheel_speed = forward_velocity / radius;
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
            let suspension_length = node.y - height - self.suspension.wheels[i].tire_radius;
            let previous_length = self.wheels[i]
                .previous_suspension_length
                .unwrap_or(suspension_length);
            // A height-field/rut sample can change discontinuously at cell
            // boundaries. Bound the derived damper shaft speed to a severe
            // but physical range so that numerical terrain edges cannot turn
            // damping into a launch impulse.
            let max_length_rate =
                (self.suspension.wheels[i].travel / self.fixed_dt.max(1e-6)).clamp(0.5, 1.5);
            let length_velocity =
                suspension_length_velocity(suspension_length, previous_length, self.fixed_dt)
                    .clamp(-max_length_rate, max_length_rate);
            let (force, compression, airborne) = raycast_wheel(
                &self.suspension.wheels[i],
                [node.x, node.y, node.z],
                length_velocity,
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
            self.wheels[i].previous_suspension_length = Some(suspension_length);
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
        let left_output_speed =
            wheel_forward_velocity[2] / self.suspension.wheels[2].tire_radius.max(0.05);
        let right_output_speed =
            wheel_forward_velocity[3] / self.suspension.wheels[3].tire_radius.max(0.05);
        let (left_torque, right_torque) =
            self.drivetrain.differential.apply_differential_with_speeds(
                drive_torque,
                rear_grip[0].max(0.01),
                rear_grip[1].max(0.01),
                left_output_speed,
                right_output_speed,
            );
        let mut transmitted_torque = 0.0;
        let mut transmitted_force = 0.0;
        let grounded_driven_wheel_count = grounded[2..]
            .iter()
            .filter(|is_grounded| **is_grounded)
            .count();
        let grounded_driven_wheels = grounded_driven_wheel_count.max(1) as f64;
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
                let requested_force = torque / wheel_radius;
                // A dissipative driveline torque may stop the chassis during
                // this explicit step, but it may not push it through zero.
                // Propulsive torque (same sign as forward velocity, or from
                // rest) remains limited only by the friction circle.
                let stopping_force = mass * forward_velocity.abs()
                    / self.fixed_dt.max(1e-6)
                    / grounded_driven_wheels;
                let dissipative_limit = if requested_force * forward_velocity < 0.0 {
                    stopping_force
                } else {
                    f64::INFINITY
                };
                let force_forward = requested_force.clamp(
                    -longitudinal_limit.min(dissipative_limit),
                    longitudinal_limit.min(dissipative_limit),
                );
                transmitted_torque += force_forward * wheel_radius;
                transmitted_force += force_forward;
                self.apply_force(
                    i,
                    force_forward * wheel_forward_axes[i][0],
                    0.0,
                    force_forward * wheel_forward_axes[i][1],
                );
            }
        }
        if grounded_driven_wheel_count > 0 {
            let rolling_speed = [(2usize, left_output_speed), (3usize, right_output_speed)]
                .into_iter()
                .filter(|(index, _)| grounded[*index])
                .map(|(_, speed)| speed)
                .sum::<f64>()
                / grounded_driven_wheel_count as f64;
            let average_driven_radius = [2usize, 3usize]
                .into_iter()
                .filter(|index| grounded[*index])
                .map(|index| self.suspension.wheels[index].tire_radius.max(0.05))
                .sum::<f64>()
                / grounded_driven_wheel_count as f64;
            let predicted_rolling_speed =
                rolling_speed + transmitted_force / mass * self.fixed_dt / average_driven_radius;
            self.drivetrain.apply_grounded_wheel_response(
                drive_torque,
                transmitted_torque,
                predicted_rolling_speed,
                self.fixed_dt,
                mass,
            );
        } else {
            self.drivetrain
                .apply_wheel_reaction_torque(transmitted_torque, self.fixed_dt, mass);
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
}
