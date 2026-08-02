use std::time::Instant;

use kemudi_engine::PhysicsWorld;

fn main() {
    let mut world = PhysicsWorld::new();
    for (id, (x, y, z)) in [
        (-0.7, 0.5, -0.6),
        (0.7, 0.5, -0.6),
        (-0.7, 0.5, 0.6),
        (0.7, 0.5, 0.6),
        (-0.7, 1.0, -0.6),
        (0.7, 1.0, -0.6),
        (-0.7, 1.0, 0.6),
        (0.7, 1.0, 0.6),
    ]
    .into_iter()
    .enumerate()
    {
        world.add_node(id, x, y, z, 50.0, false);
    }
    for (id, (a, b)) in [
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
    .into_iter()
    .enumerate()
    {
        world.add_beam(id, a, b, 10_000.0, 0.5, 100_000.0);
    }
    world.set_terrain_profile(1);
    world.set_controls(0.1, 0.8, 0.0, 0.0, false, false, false, true);

    let mut samples = Vec::with_capacity(500);
    for _ in 0..500 {
        let started = Instant::now();
        world.step(1.0 / 60.0);
        samples.push(started.elapsed().as_secs_f64() * 1000.0);
    }
    samples.sort_by(|a, b| a.total_cmp(b));
    let percentile = |p: f64| samples[((samples.len() - 1) as f64 * p) as usize];
    println!(
        "physics step p50={:.3}ms p95={:.3}ms samples={}",
        percentile(0.50),
        percentile(0.95),
        samples.len()
    );
}
