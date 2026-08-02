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
        w.add_node(
            id,
            x,
            y + 0.67,
            z,
            if id < 4 {
                45.0
            } else if id < 8 {
                25.0
            } else {
                15.0
            },
            false,
        );
    }
    let beams = [
        (0, 1, 14000.0, 2800.0),
        (2, 3, 14000.0, 2800.0),
        (4, 5, 14000.0, 2800.0),
        (6, 7, 14000.0, 2800.0),
        (8, 9, 10000.0, 1800.0),
        (10, 11, 10000.0, 1800.0),
        (0, 2, 12000.0, 2400.0),
        (1, 3, 12000.0, 2400.0),
        (4, 6, 12000.0, 2400.0),
        (5, 7, 12000.0, 2400.0),
        (0, 4, 16000.0, 3200.0),
        (1, 5, 16000.0, 3200.0),
        (2, 6, 16000.0, 3200.0),
        (3, 7, 16000.0, 3200.0),
        (4, 8, 10000.0, 2000.0),
        (5, 9, 10000.0, 2000.0),
        (6, 10, 10000.0, 2000.0),
        (7, 11, 10000.0, 2000.0),
        (8, 10, 8000.0, 1500.0),
        (9, 11, 8000.0, 1500.0),
        (0, 3, 8000.0, 1600.0),
        (1, 2, 8000.0, 1600.0),
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
    w.set_controls(0.0, 1.0, 0.0, 0.0, false, false, false, true);
    for _ in 0..240 {
        w.step(1.0 / 60.0);
    }
    assert!(
        w.telemetry[T_SPEED_MPS] > 6.0,
        "premium car must achieve meaningful road speed; got {} m/s",
        w.telemetry[T_SPEED_MPS]
    );
    assert!(w.telemetry[T_GEAR] >= 3.0);
    assert_eq!(w.telemetry[T_BROKEN_BEAMS], 0.0);
    for index in 4..12 {
        let reference = index - 4;
        let expected_y =
            w.nodes[reference].y + w.rest_positions[index][1] - w.rest_positions[reference][1];
        assert!(
            (w.nodes[index].y - expected_y).abs() < 1e-6,
            "attachment index={index} actual={} expected={expected_y}",
            w.nodes[index].y
        );
    }
    let initial_heading_offset =
        (w.nodes[0].x + w.nodes[1].x) * 0.5 - (w.nodes[2].x + w.nodes[3].x) * 0.5;
    w.set_controls(1.0, 0.45, 0.0, 0.0, false, false, false, true);
    for _ in 0..120 {
        w.step(1.0 / 60.0);
    }
    let final_heading_offset =
        (w.nodes[0].x + w.nodes[1].x) * 0.5 - (w.nodes[2].x + w.nodes[3].x) * 0.5;
    assert!(
        final_heading_offset > initial_heading_offset,
        "premium steering must yaw toward positive x: {initial_heading_offset} -> {final_heading_offset}"
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
