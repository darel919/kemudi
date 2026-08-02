use crate::math::{terrain_cell, terrain_height_for_profile};
use crate::terrain_contact::{surface_presets, TerrainContact};
use crate::types::{PhysicsWorld, TERRAIN_GRID_SIZE};

impl PhysicsWorld {
    pub(crate) fn terrain_height(&self, x: f64, z: f64) -> f64 {
        let base_height = if self.terrain_segments >= 1 && !self.terrain_heights.is_empty() {
            let segments = self.terrain_segments;
            let gx = ((x / self.terrain_width + 0.5) * segments as f64).clamp(0.0, segments as f64);
            let gz = ((z / self.terrain_depth + 0.5) * segments as f64).clamp(0.0, segments as f64);
            let ix = gx.floor() as usize;
            let iz = gz.floor() as usize;
            let x1 = (ix + 1).min(segments);
            let z1 = (iz + 1).min(segments);
            let fx = gx - ix as f64;
            let fz = gz - iz as f64;
            let stride = segments + 1;
            let h00 = self.terrain_heights[iz * stride + ix];
            let h10 = self.terrain_heights[iz * stride + x1];
            let h01 = self.terrain_heights[z1 * stride + ix];
            let h11 = self.terrain_heights[z1 * stride + x1];
            let lower = h00 + (h10 - h00) * fx;
            let upper = h01 + (h11 - h01) * fx;
            lower + (upper - lower) * fz
        } else {
            terrain_height_for_profile(self.terrain_profile, x, z)
        };
        // Rut depth is stored as a positive depression depth.
        base_height - self.rut_depth_at(x, z)
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
        // `ground_friction` is the map-authored base coefficient for the
        // terrain outside an explicit road surface. Keep the selected preset
        // for hardness/deformability/rolling behavior, but do not silently
        // fall back to asphalt friction when a map supplies another value.
        surface.base_friction = self.ground_friction;
        surface.roughness = self.surface_roughness;
        let mut moisture = self.surface_moisture;
        let mut compactness = self.surface_compactness;
        if let Some(road) = self.nearest_road_surface(x, z) {
            surface.id = road.surface_id;
            surface.base_friction = road.friction;
            surface.roughness = road.roughness;
            surface.moisture_factor = road.moisture;
            surface.rolling_resistance = (0.01 + road.roughness * 0.04).clamp(0.0, 1.0);
            surface.deformability = 1.0 - road.compactness;
            moisture = road.moisture;
            compactness = road.compactness;
        }
        let normal = self.terrain_normal(x, z);
        TerrainContact {
            surface,
            normal,
            slope_angle: normal[1].acos(),
            moisture,
            compactness,
            rut_depth: 0.0,
        }
    }

    fn nearest_road_surface(&self, x: f64, z: f64) -> Option<&crate::types::RoadSurface> {
        let mut nearest: Option<(&crate::types::RoadSurface, f64)> = None;
        for road in &self.road_surfaces {
            for segment in road.points.windows(2) {
                let [ax, az] = segment[0];
                let [bx, bz] = segment[1];
                let dx = bx - ax;
                let dz = bz - az;
                let length_sq = dx * dx + dz * dz;
                let t = if length_sq > 1e-12 {
                    ((x - ax) * dx + (z - az) * dz) / length_sq
                } else {
                    0.0
                };
                let t = t.clamp(0.0, 1.0);
                let px = ax + dx * t;
                let pz = az + dz * t;
                let distance_sq = (x - px).powi(2) + (z - pz).powi(2);
                if distance_sq <= (road.width * 0.5).powi(2)
                    && nearest
                        .map(|(_, current)| distance_sq < current)
                        .unwrap_or(true)
                {
                    nearest = Some((road, distance_sq));
                }
            }
        }
        nearest.map(|(road, _)| road)
    }
}
