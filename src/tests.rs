use super::*;
use crate::math::distance;

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

fn layered_car_world() -> PhysicsWorld {
    let mut w = PhysicsWorld::new();
    let positions = [
        (-0.75, 0.0, -0.6),
        (0.75, 0.0, -0.6),
        (-0.75, 0.0, 0.6),
        (0.75, 0.0, 0.6),
        (-0.75, 0.45, -0.6),
        (0.75, 0.45, -0.6),
        (-0.75, 0.45, 0.6),
        (0.75, 0.45, 0.6),
        (-0.6, 0.7, -0.3),
        (0.6, 0.7, -0.3),
        (-0.6, 0.7, 0.3),
        (0.6, 0.7, 0.3),
    ];
    for (id, (x, y, z)) in positions.into_iter().enumerate() {
        w.add_node(id, x, y, z, if id < 8 { 25.0 } else { 15.0 }, false);
    }
    let beams = [
        (0, 1),
        (2, 3),
        (4, 5),
        (6, 7),
        (8, 9),
        (10, 11),
        (0, 2),
        (1, 3),
        (4, 6),
        (5, 7),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
        (4, 8),
        (5, 9),
        (6, 10),
        (7, 11),
        (8, 10),
        (9, 11),
    ];
    for (id, (a, b)) in beams.into_iter().enumerate() {
        w.add_beam(id, a, b, 12_000.0, 0.5, 100_000.0);
    }
    for (a, b, c) in [
        (0, 1, 3),
        (0, 3, 2),
        (4, 6, 7),
        (4, 7, 5),
        (4, 5, 9),
        (4, 9, 8),
        (6, 10, 11),
        (6, 11, 7),
        (8, 9, 11),
        (8, 11, 10),
    ] {
        w.add_triangle(a, b, c);
    }
    w
}

fn configured_layered_world(fixed_dt: f64) -> PhysicsWorld {
    let mut w = layered_car_world();
    w.fixed_dt = fixed_dt;
    w.max_substeps = 16;
    w.configure_runtime(
        850.0,
        8000.0,
        8200.0,
        1.1,
        25.0,
        &[0.0, 1000.0, 3000.0, 5000.0, 8000.0],
        &[150.0, 300.0, 500.0, 520.0, 350.0],
        &[3.8, 2.4, 1.7, 1.2, 0.9, 0.7],
        3.4,
        -3.0,
        1,
        0.1,
        2,
        0.55,
        &[36_000.0, 36_000.0, 40_000.0, 40_000.0],
        &[5_000.0, 5_000.0, 5_500.0, 5_500.0],
        &[3_200.0, 3_200.0, 3_500.0, 3_500.0],
        &[0.33; 4],
        &[0.18; 4],
        &[0.34; 4],
        2_200.0,
        500_000.0,
        &[1; 4],
        &[34.0; 4],
        65.0,
        0.015,
        0.006,
        true,
        true,
        true,
        false,
        false,
    );
    let lift = (0..4)
        .map(|index| {
            let node = w.nodes[index];
            let wheel = &w.suspension.wheels[index];
            w.terrain_height(node.x, node.z) + wheel.rest_length + wheel.tire_radius - node.y
        })
        .fold(0.0, f64::max)
        .max(0.34);
    for node in &mut w.nodes {
        node.y += lift;
    }
    for rest in &mut w.rest_positions {
        rest[1] += lift;
    }
    w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, false);
    for _ in 0..(2.0 / fixed_dt).round() as usize {
        w.step(fixed_dt);
    }
    w
}

fn max_beam_error(w: &PhysicsWorld) -> f64 {
    w.beams
        .iter()
        .filter(|beam| !beam.broken)
        .map(|beam| {
            let a = w.nodes[beam.node_a];
            let b = w.nodes[beam.node_b];
            (distance(a.x, a.y, a.z, b.x, b.y, b.z) - beam.length).abs()
        })
        .fold(0.0, f64::max)
}

fn kinetic_energy(w: &PhysicsWorld) -> f64 {
    let chassis = w
        .nodes
        .iter()
        .filter(|node| !node.fixed)
        .map(|node| 0.5 * node.mass * (node.vx * node.vx + node.vy * node.vy + node.vz * node.vz))
        .sum::<f64>();
    let mass = w
        .nodes
        .iter()
        .filter(|node| !node.fixed)
        .map(|node| node.mass)
        .sum::<f64>();
    chassis + 0.5 * (mass * 0.01) * w.drivetrain.wheel_speed.powi(2)
}

fn layer_yaw(
    w: &PhysicsWorld,
    front_left: usize,
    front_right: usize,
    rear_left: usize,
    rear_right: usize,
) -> f64 {
    let front_x = (w.nodes[front_left].x + w.nodes[front_right].x) * 0.5;
    let front_z = (w.nodes[front_left].z + w.nodes[front_right].z) * 0.5;
    let rear_x = (w.nodes[rear_left].x + w.nodes[rear_right].x) * 0.5;
    let rear_z = (w.nodes[rear_left].z + w.nodes[rear_right].z) * 0.5;
    (front_x - rear_x).atan2(-(front_z - rear_z))
}

fn max_wheel_mount_frame_error(w: &PhysicsWorld) -> f64 {
    let mount_count = 4.min(w.nodes.len()).min(w.rest_positions.len());
    let mut maximum = 0.0_f64;
    for left in 0..mount_count {
        for right in (left + 1)..mount_count {
            let rest_distance = distance(
                w.rest_positions[left][0],
                w.rest_positions[left][1],
                w.rest_positions[left][2],
                w.rest_positions[right][0],
                w.rest_positions[right][1],
                w.rest_positions[right][2],
            );
            let current_distance = distance(
                w.nodes[left].x,
                w.nodes[left].y,
                w.nodes[left].z,
                w.nodes[right].x,
                w.nodes[right].y,
                w.nodes[right].z,
            );
            maximum = maximum.max((current_distance - rest_distance).abs());
        }
    }
    maximum
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
    let oil_pressure_before = w.lubrication.oil_pressure;
    w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, false);
    w.step(1.0 / 60.0);
    assert_eq!(w.telemetry[T_RPM], 0.0);
    assert_eq!(w.telemetry[T_DRIVE_TORQUE], 0.0);
    assert!(
        w.lubrication.oil_pressure < oil_pressure_before,
        "engine-off oil pressure must decay without pump output"
    );
    assert_eq!(w.telemetry[T_ENGINE_WARNING], 0.0);
    assert_eq!(
        w.fuel.current_level, fuel_before,
        "engine-off vehicle must not burn fuel"
    );
}

#[test]
fn engine_off_stationary_vehicle_remains_coherent_for_ten_seconds() {
    let mut w = configured_layered_world(1.0 / 120.0);
    let initial_energy = kinetic_energy(&w);
    for _ in 0..1200 {
        w.step(1.0 / 120.0);
    }
    assert!(
        w.telemetry[T_SPEED_MPS] < 0.02,
        "engine-off drift was {} m/s",
        w.telemetry[T_SPEED_MPS]
    );
    assert!(w.drivetrain.wheel_speed.abs() * w.drivetrain.wheel_radius < 0.02);
    assert!(
        max_beam_error(&w) < 0.02,
        "constraint error was {} m",
        max_beam_error(&w)
    );
    assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
    assert!(
        kinetic_energy(&w) <= initial_energy + 1.0,
        "unpowered kinetic energy increased"
    );
}

#[test]
fn engine_start_at_zero_throttle_preserves_startup_invariants_for_ten_seconds() {
    #[derive(Debug)]
    struct StartupSample {
        step: usize,
        raw_throttle: f64,
        processed_throttle: f64,
        rpm: f64,
        drive_torque: f64,
        wheel_linear_speed: f64,
        ground_speed: f64,
        slip_ratio: f64,
        kinetic_energy: f64,
        constraint_error: f64,
        node_positions: Vec<[f64; 3]>,
        node_velocities: Vec<[f64; 3]>,
        node_forces: Vec<f64>,
    }

    let dt = 1.0 / 120.0;
    let mut w = configured_layered_world(dt);
    let initial_energy = kinetic_energy(&w);
    let mut trace = Vec::with_capacity(1200);
    w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, true);
    for step in 1..=1200 {
        w.step(dt);
        trace.push(StartupSample {
            step,
            raw_throttle: w.controls.throttle,
            processed_throttle: w.drivetrain.throttle_input,
            rpm: w.drivetrain.engine.rpm,
            drive_torque: w.drivetrain.last_drive_torque,
            wheel_linear_speed: w.drivetrain.wheel_speed * w.drivetrain.wheel_radius,
            ground_speed: w.telemetry[T_SPEED_MPS],
            slip_ratio: w.telemetry[T_SLIP_RATIO],
            kinetic_energy: kinetic_energy(&w),
            constraint_error: max_beam_error(&w),
            node_positions: w
                .nodes
                .iter()
                .map(|node| [node.x, node.y, node.z])
                .collect(),
            node_velocities: w
                .nodes
                .iter()
                .map(|node| [node.vx, node.vy, node.vz])
                .collect(),
            node_forces: w.nodes.iter().map(|node| node.last_force).collect(),
        });
    }

    let invalid = trace.iter().find(|sample| {
        sample.raw_throttle != 0.0
            || sample.processed_throttle != 0.0
            || sample.drive_torque * sample.wheel_linear_speed > 1e-9
            || sample.rpm > w.drivetrain.engine.idle_rpm + 50.0
            || sample.ground_speed > 0.05
            || sample.wheel_linear_speed.abs() > 0.05
            || sample.constraint_error > 0.02
            || !sample.slip_ratio.is_finite()
            || sample
                .node_positions
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
            || sample
                .node_velocities
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
            || sample.node_forces.iter().any(|value| !value.is_finite())
    });
    if let Some(sample) = invalid {
        panic!(
            "first invalid startup sample at step {}: {sample:#?}",
            sample.step
        );
    }
    let final_sample = trace.last().unwrap();
    assert!((final_sample.rpm - w.drivetrain.engine.idle_rpm).abs() < 1.0);
    assert!(final_sample.kinetic_energy <= initial_energy + 1.0);
    assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
}

#[test]
fn neutral_allows_engine_revs_without_wheel_or_chassis_torque() {
    let mut w = configured_layered_world(1.0 / 120.0);
    w.drivetrain.set_gear(0);
    w.set_controls(0.0, 0.7, 0.0, 0.0, false, false, false, true);
    for _ in 0..600 {
        w.step(1.0 / 120.0);
    }
    assert!(w.drivetrain.engine.rpm > w.drivetrain.engine.idle_rpm + 1000.0);
    assert!(w.drivetrain.last_drive_torque.abs() < 1e-9);
    assert!(w.drivetrain.wheel_speed.abs() * w.drivetrain.wheel_radius < 0.05);
    assert!(w.telemetry[T_SPEED_MPS] < 0.05);
}

#[test]
fn zero_throttle_in_gear_engine_braking_opposes_motion() {
    let mut w = configured_layered_world(1.0 / 120.0);
    for node in &mut w.nodes {
        node.vz = -5.0;
    }
    w.drivetrain.wheel_speed = 5.0 / w.drivetrain.wheel_radius;
    w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, true);
    w.step(1.0 / 120.0);
    assert!(
        w.drivetrain.last_drive_torque < 0.0,
        "engine braking must oppose positive wheel speed"
    );
    assert!(
        w.drivetrain.last_drive_torque.abs()
            <= w.drivetrain.engine.engine_braking * w.drivetrain.transmission.total_ratio().abs()
                + 1e-9
    );
}

#[test]
fn fixed_step_variants_keep_zero_throttle_startup_stationary() {
    let mut results = Vec::new();
    for fixed_dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
        let mut w = configured_layered_world(fixed_dt);
        w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, true);
        for _ in 0..(10.0 / fixed_dt).round() as usize {
            w.step(fixed_dt);
        }
        results.push((
            w.telemetry[T_SPEED_MPS],
            w.drivetrain.engine.rpm,
            max_beam_error(&w),
        ));
    }
    for (speed, rpm, constraint_error) in results {
        assert!(speed < 0.05, "fixed-step startup drift was {speed} m/s");
        assert!(
            (rpm - 850.0).abs() < 1.0,
            "fixed-step startup rpm was {rpm}"
        );
        assert!(
            constraint_error < 0.02,
            "fixed-step constraint error was {constraint_error} m"
        );
    }
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
fn steering_input_is_applied_during_launch() {
    let mut w = car_world();
    w.set_controls(1.0, 0.35, 0.0, 0.0, false, false, false, true);
    for _ in 0..60 {
        w.step(1.0 / 60.0);
    }
    assert!(
        w.telemetry[T_STEERING] > 0.05,
        "steering input must reach the wheel-angle state"
    );
    let front_x = (w.nodes[0].x + w.nodes[1].x) * 0.5;
    let rear_x = (w.nodes[2].x + w.nodes[3].x) * 0.5;
    assert!(
        front_x > rear_x,
        "the chassis must begin yawing during a low-speed steering input"
    );
}

#[test]
fn layered_body_spawns_above_terrain_and_stays_beam_connected() {
    let mut w = layered_car_world();
    let lift = (0..4)
        .map(|index| {
            let node = w.nodes[index];
            let wheel = &w.suspension.wheels[index];
            w.terrain_height(node.x, node.z) + wheel.rest_length + wheel.tire_radius - node.y
        })
        .fold(0.0, f64::max);
    for node in &mut w.nodes {
        node.y += lift.max(0.34);
    }
    w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, false);
    for _ in 0..120 {
        w.step(1.0 / 60.0);
    }
    assert!(w.nodes.iter().take(4).all(|node| node.y > 0.2));
    assert!(w.nodes.iter().skip(4).all(|node| node.y > 0.2));
    let min_body_y = w
        .nodes
        .iter()
        .skip(4)
        .map(|node| node.y)
        .fold(f64::INFINITY, f64::min);
    let broken_beams = w.beams.iter().filter(|beam| beam.broken).count();
    let vertical_lengths = [10usize, 11, 12, 13]
        .iter()
        .map(|&index| {
            let beam = &w.beams[index];
            let a = w.nodes[beam.node_a];
            let b = w.nodes[beam.node_b];
            ((b.x - a.x).powi(2) + (b.y - a.y).powi(2) + (b.z - a.z).powi(2)).sqrt()
        })
        .collect::<Vec<_>>();
    assert!(
        min_body_y > 0.2,
        "upper cage collapsed into terrain; min body y={min_body_y}, wheel y={:?}, body y={:?}, vertical beams={vertical_lengths:?}, broken beams={broken_beams}",
        w.nodes.iter().take(4).map(|node| node.y).collect::<Vec<_>>(),
        w.nodes.iter().skip(4).map(|node| node.y).collect::<Vec<_>>()
    );
    assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
}

#[test]
fn body_attachment_preserves_a_rigidly_rotated_chassis_frame() {
    let mut w = layered_car_world();
    let angle: f64 = 0.2;
    let cosine = angle.cos();
    let sine = angle.sin();
    for node in &mut w.nodes {
        let x = node.x;
        let y = node.y;
        node.x = x * cosine - y * sine;
        node.y = x * sine + y * cosine;
    }
    let before = w.get_positions_flat();
    w.solve_xpbd_constraints();
    let after = w.get_positions_flat();
    let max_projection = before
        .iter()
        .zip(after.iter())
        .map(|(before, after)| (after - before).abs())
        .fold(0.0, f64::max);
    assert!(
        max_projection < 1e-9,
        "a rigid chassis rotation must not be mistaken for body separation: {max_projection} m"
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
        2200.0,
        500000.0,
        &[1; 4],
        &[32.0; 4],
        60.0,
        0.01,
        0.0005,
        false,
        true,
        false,
        false,
        false,
    );
    w.set_automatic_shift_schedule(5000.0, 2200.0);
    assert_eq!(w.drivetrain.transmission.current_gear, 1);
    w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, true);
    let mut previous_gear = w.drivetrain.transmission.current_gear;
    let mut highest_gear = previous_gear;
    for _ in 0..1200 {
        w.step(1.0 / 60.0);
        let current_gear = w.drivetrain.transmission.current_gear;
        assert!(
            (current_gear - previous_gear).abs() <= 1,
            "automatic transmission must shift sequentially: {previous_gear} -> {current_gear}"
        );
        previous_gear = current_gear;
        highest_gear = highest_gear.max(current_gear);
    }
    assert!(
        w.telemetry[T_SPEED_MPS] > 5.0,
        "a configured powered car must achieve meaningful road speed; got {} m/s in gear {} at {} rpm",
        w.telemetry[T_SPEED_MPS],
        w.telemetry[T_GEAR],
        w.telemetry[T_RPM]
    );
    assert!(
        w.telemetry[T_GEAR] > 1.0,
        "automatic drivetrain should upshift while accelerating"
    );
    assert!(
        highest_gear >= 3,
        "automatic TCM must progress beyond 2nd gear under sustained acceleration"
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
fn premium_layout_stays_attached_under_launch() {
    let mut w = PhysicsWorld::new();
    let positions = [
        (-0.8, 0.0, -1.35),
        (0.8, 0.0, -1.35),
        (-0.8, 0.0, 1.35),
        (0.8, 0.0, 1.35),
        (-0.8, 0.45, -1.35),
        (0.8, 0.45, -1.35),
        (-0.8, 0.45, 1.35),
        (0.8, 0.45, 1.35),
        (-0.65, 0.7, -0.75),
        (0.65, 0.7, -0.75),
        (-0.65, 0.7, 0.75),
        (0.65, 0.7, 0.75),
    ];
    for (id, (x, y, z)) in positions.into_iter().enumerate() {
        w.add_node(
            id,
            x,
            y + 0.67,
            z,
            if id < 4 {
                200.0
            } else if id < 8 {
                120.0
            } else {
                55.0
            },
            false,
        );
    }
    let beams = [
        (0, 1, 1_400_000.0, 112_000.0),
        (2, 3, 1_400_000.0, 112_000.0),
        (4, 5, 1_400_000.0, 112_000.0),
        (6, 7, 1_400_000.0, 112_000.0),
        (8, 9, 1_000_000.0, 72_000.0),
        (10, 11, 1_000_000.0, 72_000.0),
        (0, 2, 1_200_000.0, 96_000.0),
        (1, 3, 1_200_000.0, 96_000.0),
        (4, 6, 1_200_000.0, 96_000.0),
        (5, 7, 1_200_000.0, 96_000.0),
        (0, 4, 1_600_000.0, 128_000.0),
        (1, 5, 1_600_000.0, 128_000.0),
        (2, 6, 1_600_000.0, 128_000.0),
        (3, 7, 1_600_000.0, 128_000.0),
        (4, 8, 1_000_000.0, 80_000.0),
        (5, 9, 1_000_000.0, 80_000.0),
        (6, 10, 1_000_000.0, 80_000.0),
        (7, 11, 1_000_000.0, 80_000.0),
        (8, 10, 800_000.0, 60_000.0),
        (9, 11, 800_000.0, 60_000.0),
        (0, 3, 800_000.0, 64_000.0),
        (1, 2, 800_000.0, 64_000.0),
    ];
    for (id, (a, b, stiffness, strength)) in beams.into_iter().enumerate() {
        w.add_beam(id, a, b, stiffness, 0.5, strength * 0.775);
    }
    for (a, b, c) in [
        (0, 1, 3),
        (0, 3, 2),
        (4, 6, 7),
        (4, 7, 5),
        (4, 5, 9),
        (4, 9, 8),
        (6, 10, 11),
        (6, 11, 7),
        (8, 9, 11),
        (8, 11, 10),
    ] {
        w.add_triangle(a, b, c);
    }
    w.configure_runtime(
        850.0,
        8000.0,
        8200.0,
        1.1,
        25.0,
        &[
            0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0,
        ],
        &[
            150.0, 300.0, 420.0, 500.0, 540.0, 520.0, 480.0, 420.0, 350.0,
        ],
        &[3.8, 2.4, 1.7, 1.2, 0.9, 0.7],
        3.4,
        -3.0,
        1,
        0.1,
        2,
        0.55,
        &[36000.0, 36000.0, 40000.0, 40000.0],
        &[5000.0, 5000.0, 5500.0, 5500.0],
        &[3200.0, 3200.0, 3500.0, 3500.0],
        &[0.33; 4],
        &[0.18; 4],
        &[0.34; 4],
        2200.0,
        500000.0,
        &[1; 4],
        &[34.0; 4],
        65.0,
        0.015,
        0.006,
        true,
        true,
        true,
        true,
        true,
    );
    w.set_automatic_shift_schedule(5000.0, 2200.0);
    assert!((w.steering_config.wheelbase - 2.7).abs() < 1e-9);
    assert!((w.steering_config.track_width - 1.6).abs() < 1e-9);
    w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, false);
    for _ in 0..120 {
        w.step(1.0 / 60.0);
    }
    w.set_controls(0.0, 0.0, 0.0, 0.0, false, false, false, true);
    for _ in 0..600 {
        w.step(1.0 / 60.0);
    }
    println!(
        "zero-input premium: speed={} rpm={} gear={} torque={} wheel={} slip={} max_constraint_error={} rear_vz=({}, {}) positions={:?}",
        w.telemetry[T_SPEED_MPS],
        w.telemetry[T_RPM],
        w.telemetry[T_GEAR],
        w.telemetry[T_DRIVE_TORQUE],
        w.drivetrain.wheel_speed,
        w.telemetry[T_SLIP_RATIO],
        max_beam_error(&w),
        w.nodes[2].vz,
        w.nodes[3].vz,
        w.nodes.iter().map(|node| [node.x, node.y, node.z]).collect::<Vec<_>>(),
    );
    assert!(w.telemetry[T_SPEED_MPS] < 0.05);
    assert!((w.telemetry[T_RPM] - 850.0).abs() < 1.0);
    assert!(
        w.telemetry[T_DRIVE_TORQUE] * w.drivetrain.wheel_speed <= 1e-9,
        "zero-throttle driveline torque must remain dissipative"
    );
    assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
    w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, true);
    for _ in 0..1200 {
        w.step(1.0 / 60.0);
        if w.telemetry[T_GEAR] >= 6.0 && w.telemetry[T_SPEED_MPS] >= 14.0 {
            break;
        }
    }
    assert!(
        w.telemetry[T_SPEED_MPS] >= 14.0,
        "premium car must reach the reported failure speed; got {} m/s",
        w.telemetry[T_SPEED_MPS]
    );
    assert!(w.telemetry[T_GEAR] >= 6.0);
    assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
    for index in 4..12 {
        let reference = index - 4;
        let rest_offset = distance(
            w.rest_positions[index][0],
            w.rest_positions[index][1],
            w.rest_positions[index][2],
            w.rest_positions[reference][0],
            w.rest_positions[reference][1],
            w.rest_positions[reference][2],
        );
        let current_offset = distance(
            w.nodes[index].x,
            w.nodes[index].y,
            w.nodes[index].z,
            w.nodes[reference].x,
            w.nodes[reference].y,
            w.nodes[reference].z,
        );
        assert!(
            (current_offset - rest_offset).abs() < 0.002,
            "attachment index={index} current={current_offset} rest={rest_offset}",
        );
    }
    let initial_yaw = layer_yaw(&w, 0, 1, 2, 3);
    w.set_controls(1.0, 0.45, 0.0, 0.0, false, false, false, true);
    let mut maximum_mount_distance_error = max_wheel_mount_frame_error(&w);
    for _ in 0..120 {
        w.step(1.0 / 60.0);
        maximum_mount_distance_error =
            maximum_mount_distance_error.max(max_wheel_mount_frame_error(&w));
    }
    let chassis_yaw = layer_yaw(&w, 0, 1, 2, 3);
    let body_yaw = layer_yaw(&w, 4, 5, 6, 7);
    println!(
        "premium steering: initial_yaw={initial_yaw} chassis_yaw={chassis_yaw} body_yaw={body_yaw} speed={} gear={} steering={} broken_beams={} maximum_mount_distance_error={}",
        w.telemetry[T_SPEED_MPS],
        w.telemetry[T_GEAR],
        w.telemetry[T_STEERING],
        w.telemetry[T_BROKEN_BEAMS],
        maximum_mount_distance_error,
    );
    assert!(
        chassis_yaw > initial_yaw + 0.05,
        "positive steering did not produce a meaningful right yaw"
    );
    assert!(
        (body_yaw - chassis_yaw).abs() < 0.01,
        "body yaw separated from chassis yaw"
    );
    assert_eq!(
        w.telemetry[T_BROKEN_BEAMS], 0.0,
        "ordinary acceleration and steering must not break the chassis"
    );
    assert!(
        maximum_mount_distance_error < 0.01,
        "wheel mounts separated during steering by {maximum_mount_distance_error} m"
    );
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
