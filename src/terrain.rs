use crate::math::{terrain_cell, terrain_height_for_profile};
use crate::terrain_contact::{surface_presets, TerrainContact};
use crate::types::{PhysicsWorld, TERRAIN_GRID_SIZE};

impl PhysicsWorld {
    pub(crate) fn terrain_height(&self, x: f64, z: f64) -> f64 {
        terrain_height_for_profile(self.terrain_profile, x, z) + self.rut_depth_at(x, z)
    }

    pub(crate) fn rut_depth_at(&self, x: f64, z: f64) -> f64 {
        let cell = terrain_cell(x, z);
        self.terrain_ruts[cell.1 * TERRAIN_GRID_SIZE + cell.0]
    }

    pub(crate) fn terrain_normal(&self, x: f64, z: f64) -> [f64; 3] {
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

    pub(crate) fn deposit_rut(
        &mut self,
        x: f64,
        z: f64,
        load: f64,
        slip_ratio: f64,
        deformability: f64,
    ) {
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

    /// Compute surface contact properties at a world position.
    /// Uses data-driven surface properties set from map config — no hardcoded profile checks.
    pub(crate) fn terrain_contact(&self, x: f64, z: f64) -> TerrainContact {
        let presets = surface_presets();
        let preset_idx = (self.surface_preset_index as usize).min(presets.len() - 1);
        let mut surface = presets[preset_idx].clone();
        surface.roughness = self.surface_roughness;
        let normal = self.terrain_normal(x, z);
        TerrainContact {
            surface,
            normal,
            slope_angle: normal[1].acos(),
            moisture: self.surface_moisture,
            compactness: self.surface_compactness,
            rut_depth: 0.0,
        }
    }
}
