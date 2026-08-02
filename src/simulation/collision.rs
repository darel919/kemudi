//! Terrain collision, impact history, and persistent structural damage.

use crate::math::classify_damage_zone;
use crate::types::*;

const BODY_NODE_CLEARANCE: f64 = 0.2;

fn normalize3(value: [f64; 3]) -> Option<[f64; 3]> {
    let length = (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt();
    (length > 1e-8).then(|| [value[0] / length, value[1] / length, value[2] / length])
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

impl PhysicsWorld {
    pub(crate) fn collide_with_static_geometry(&mut self) {
        let collision_boxes = self.collision_boxes.clone();
        let collision_spheres = self.collision_spheres.clone();
        let boundaries = self.boundaries.clone();
        for index in 0..self.nodes.len() {
            if self.nodes[index].fixed {
                continue;
            }
            for collision in collision_boxes.iter().copied() {
                self.resolve_box_contact(index, collision);
            }
            for collision in collision_spheres.iter().copied() {
                self.resolve_sphere_contact(index, collision);
            }
            for boundary in boundaries.iter().copied() {
                self.resolve_boundary_contact(index, boundary);
            }
        }
    }

    fn resolve_box_contact(&mut self, index: usize, collision: StaticCollisionBox) {
        let node = self.nodes[index];
        let relative = [
            node.x - collision.center[0],
            node.y - collision.center[1],
            node.z - collision.center[2],
        ];
        let penetration = [
            collision.half_extents[0] - relative[0].abs(),
            collision.half_extents[1] - relative[1].abs(),
            collision.half_extents[2] - relative[2].abs(),
        ];
        if penetration.iter().any(|value| *value <= 0.0) {
            return;
        }
        let axis = if node.vx.abs() + node.vy.abs() + node.vz.abs() > 1e-6 {
            [node.vx.abs(), node.vy.abs(), node.vz.abs()]
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.total_cmp(b))
                .map(|(axis, _)| axis)
                .unwrap_or(1)
        } else {
            penetration
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.total_cmp(b))
                .map(|(axis, _)| axis)
                .unwrap_or(1)
        };
        let mut normal = [0.0; 3];
        let velocity_axis = [node.vx, node.vy, node.vz][axis];
        normal[axis] = if velocity_axis.abs() > 1e-8 {
            -velocity_axis.signum()
        } else if relative[axis] >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let correction = penetration[axis];
        self.nodes[index].x += normal[0] * correction;
        self.nodes[index].y += normal[1] * correction;
        self.nodes[index].z += normal[2] * correction;
        self.apply_contact_velocity(index, normal, collision.restitution, collision.friction);
    }

    fn resolve_sphere_contact(&mut self, index: usize, collision: StaticCollisionSphere) {
        let node = self.nodes[index];
        let relative = [
            node.x - collision.center[0],
            node.y - collision.center[1],
            node.z - collision.center[2],
        ];
        let distance =
            (relative[0] * relative[0] + relative[1] * relative[1] + relative[2] * relative[2])
                .sqrt();
        if distance >= collision.radius {
            return;
        }
        let normal = if distance > 1e-8 {
            [
                relative[0] / distance,
                relative[1] / distance,
                relative[2] / distance,
            ]
        } else {
            [0.0, 1.0, 0.0]
        };
        self.nodes[index].x = collision.center[0] + normal[0] * collision.radius;
        self.nodes[index].y = collision.center[1] + normal[1] * collision.radius;
        self.nodes[index].z = collision.center[2] + normal[2] * collision.radius;
        self.apply_contact_velocity(index, normal, collision.restitution, collision.friction);
    }

    fn resolve_boundary_contact(&mut self, index: usize, boundary: StaticBoundary) {
        if self.nodes[index].y >= boundary.point[1] {
            return;
        }
        self.nodes[index].y = boundary.point[1];
        self.apply_contact_velocity(
            index,
            [0.0, 1.0, 0.0],
            boundary.restitution,
            boundary.friction,
        );
    }

    fn apply_contact_velocity(
        &mut self,
        index: usize,
        normal: [f64; 3],
        restitution: f64,
        friction: f64,
    ) {
        let node = &mut self.nodes[index];
        let normal_velocity = node.vx * normal[0] + node.vy * normal[1] + node.vz * normal[2];
        if normal_velocity < 0.0 {
            node.vx -= normal_velocity * (1.0 + restitution) * normal[0];
            node.vy -= normal_velocity * (1.0 + restitution) * normal[1];
            node.vz -= normal_velocity * (1.0 + restitution) * normal[2];
        }
        let tangent_scale = (1.0 - friction * 0.25).clamp(0.0, 1.0);
        let post_normal = node.vx * normal[0] + node.vy * normal[1] + node.vz * normal[2];
        node.vx = (node.vx - post_normal * normal[0]) * tangent_scale + post_normal * normal[0];
        node.vy = (node.vy - post_normal * normal[1]) * tangent_scale + post_normal * normal[1];
        node.vz = (node.vz - post_normal * normal[2]) * tangent_scale + post_normal * normal[2];
    }

    /// Convert a collision impulse at one node into persistent deformation of
    /// the beams that can carry that impulse. Node masses remain at their
    /// deformed positions, so center-of-mass and alignment consequences emerge
    /// from the authoritative soft-body geometry instead of a parallel damage
    /// offset model.
    pub(crate) fn apply_impact_damage(
        &mut self,
        node_index: usize,
        impact_impulse: f64,
        collision_normal: [f64; 3],
    ) -> usize {
        if node_index >= self.nodes.len() || !impact_impulse.is_finite() || impact_impulse <= 0.0 {
            return 0;
        }
        let Some(normal) = normalize3(collision_normal) else {
            return 0;
        };
        let impact_force = impact_impulse / self.fixed_dt.max(1e-6);
        let mut load_paths = Vec::new();
        for (beam_index, beam) in self.beams.iter().enumerate() {
            if beam.broken || (beam.node_a != node_index && beam.node_b != node_index) {
                continue;
            }
            let other_index = if beam.node_a == node_index {
                beam.node_b
            } else {
                beam.node_a
            };
            if other_index >= self.nodes.len() {
                continue;
            }
            let node = self.nodes[node_index];
            let other = self.nodes[other_index];
            let Some(axis) = normalize3([other.x - node.x, other.y - node.y, other.z - node.z])
            else {
                continue;
            };
            let signed_projection = dot3(axis, normal);
            let load_weight = signed_projection.abs();
            if load_weight > 1e-4 {
                load_paths.push((beam_index, signed_projection, load_weight));
            }
        }
        let total_weight = load_paths.iter().map(|(_, _, weight)| *weight).sum::<f64>();
        if total_weight <= 1e-9 {
            return 0;
        }

        let mut damaged_count = 0;
        let mut normalized_damage = 0.0;
        for (beam_index, signed_projection, load_weight) in load_paths {
            let beam = &mut self.beams[beam_index];
            let axial_force = impact_force * load_weight / total_weight;
            let yield_force = beam.strength * 0.55;
            if axial_force <= yield_force {
                continue;
            }
            damaged_count += 1;
            if axial_force > beam.strength {
                beam.broken = true;
                normalized_damage += 1.0;
                continue;
            }

            let plastic_increment = ((axial_force - yield_force) / beam.stiffness.max(1.0) * 0.2)
                .min(beam.initial_length * 0.05);
            let direction = if signed_projection > 0.0 { -1.0 } else { 1.0 };
            let previous_length = beam.length;
            beam.length = (beam.length + direction * plastic_increment)
                .clamp(beam.initial_length * 0.7, beam.initial_length * 1.3);
            normalized_damage +=
                (beam.length - previous_length).abs() / beam.initial_length.max(1e-6);
        }
        if damaged_count > 0 {
            self.body_damage = (self.body_damage
                + normalized_damage / self.beams.len().max(1) as f64)
                .clamp(0.0, 1.0);
        }
        damaged_count
    }

    pub(crate) fn collide_with_terrain(&mut self) {
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
            let terrain_height = self.terrain_height(x, z);
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
                        // Settling and ordinary suspension bottom-out may be
                        // reportable impacts without exceeding the material
                        // yield threshold of the body structure.
                        if severity >= 0.12 {
                            self.apply_impact_damage(index, impact_impulse, normal);
                        }
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
                    node.vx *= self.ground_friction;
                    node.vz *= self.ground_friction;
                }
            }
        }
    }
}
