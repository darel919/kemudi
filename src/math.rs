use crate::types::{Node, TERRAIN_EXTENT, TERRAIN_GRID_SIZE};

pub(crate) fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

pub(crate) fn terrain_height_for_profile(profile: u8, x: f64, z: f64) -> f64 {
    match profile {
        // 0 = Ground Zero: flat asphalt grid
        0 => 0.0,
        // 1 = Dragville: flat drag strip
        1 => 0.0,
        // 2 = Amazon: jungle terrain with rocks, ridges, and roughness.
        // Keep coefficients in lockstep with useTerrain.ts terrainHeight().
        2 => {
            let broad = (x * 0.008).sin() * 0.6 + (z * 0.006).cos() * 0.5;
            let ridges = ((x - z) * 0.025).sin() * 0.35 + ((x + z) * 0.018).cos() * 0.25;
            let rocks = (x * 0.08).sin() * (z * 0.06).cos() * 0.18
                + ((x * 2.1 + z * 0.7) * 0.05).sin() * 0.12;
            let rough = (x * 0.35 + z * 0.28).sin() * 0.06 + (x * 0.42 - z * 0.31).cos() * 0.04;
            (broad + ridges + rocks + rough) * 14.0 * 0.12
        }
        _ => 0.0,
    }
}

pub(crate) fn terrain_cell(x: f64, z: f64) -> (usize, usize) {
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
pub(crate) fn classify_damage_zone(x: f64, y: f64, z: f64, center_x: f64, center_z: f64) -> u8 {
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
pub(crate) fn sane_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}
pub(crate) fn value_or(values: &[f64], index: usize, fallback: f64) -> f64 {
    values
        .get(index)
        .copied()
        .filter(|v| v.is_finite())
        .unwrap_or(fallback)
}
pub(crate) fn value_or_u8(values: &[u8], index: usize, fallback: u8) -> u8 {
    values.get(index).copied().unwrap_or(fallback)
}
pub(crate) fn distance(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> f64 {
    ((bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2)).sqrt()
}
pub(crate) fn triangle_area(a: &Node, b: &Node, c: &Node) -> f64 {
    let ab = [b.x - a.x, b.y - a.y, b.z - a.z];
    let ac = [c.x - a.x, c.y - a.y, c.z - a.z];
    let cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt()
}
