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
                step, w.telemetry[T_GEAR], w.telemetry[T_RPM], w.telemetry[T_SPEED_MPS]
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
