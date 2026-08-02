//! XPBD beam, triangle-area, and body-attachment projection.

use crate::types::PhysicsWorld;

fn normalize3(value: [f64; 3]) -> Option<[f64; 3]> {
    let length = (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt();
    (length > 1e-8).then(|| [value[0] / length, value[1] / length, value[2] / length])
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn chassis_basis(points: [[f64; 3]; 4]) -> Option<[[f64; 3]; 3]> {
    let front = [
        (points[0][0] + points[1][0]) * 0.5,
        (points[0][1] + points[1][1]) * 0.5,
        (points[0][2] + points[1][2]) * 0.5,
    ];
    let rear = [
        (points[2][0] + points[3][0]) * 0.5,
        (points[2][1] + points[3][1]) * 0.5,
        (points[2][2] + points[3][2]) * 0.5,
    ];
    let left = [
        (points[0][0] + points[2][0]) * 0.5,
        (points[0][1] + points[2][1]) * 0.5,
        (points[0][2] + points[2][2]) * 0.5,
    ];
    let right = [
        (points[1][0] + points[3][0]) * 0.5,
        (points[1][1] + points[3][1]) * 0.5,
        (points[1][2] + points[3][2]) * 0.5,
    ];
    let forward = normalize3([front[0] - rear[0], front[1] - rear[1], front[2] - rear[2]])?;
    let raw_right = [right[0] - left[0], right[1] - left[1], right[2] - left[2]];
    let forward_projection = dot3(raw_right, forward);
    let right = normalize3([
        raw_right[0] - forward[0] * forward_projection,
        raw_right[1] - forward[1] * forward_projection,
        raw_right[2] - forward[2] * forward_projection,
    ])?;
    let up = normalize3([
        right[1] * forward[2] - right[2] * forward[1],
        right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0],
    ])?;
    Some([right, up, forward])
}

impl PhysicsWorld {
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

    /// Keep the authored upper cage in the rotating chassis frame. Distance
    /// beams alone form linkages: they preserve length but permit the body to
    /// shear or yaw independently of the suspension mounts. The attachment
    /// offset is therefore expressed in the authored mount basis and rebuilt
    /// in the current mount basis before XPBD projection. Broken direct beams
    /// still release their body point into the normal damage path.
    fn solve_body_attachment_constraints(&mut self) {
        let node_count = self.nodes.len().min(self.rest_positions.len());
        if node_count < 8 {
            return;
        }
        let current_mounts = std::array::from_fn(|index| {
            let node = self.nodes[index];
            [node.x, node.y, node.z]
        });
        let rest_mounts = std::array::from_fn(|index| self.rest_positions[index]);
        let (Some(current_basis), Some(rest_basis)) =
            (chassis_basis(current_mounts), chassis_basis(rest_mounts))
        else {
            return;
        };
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
            let rest_delta = [
                self.rest_positions[index][0] - self.rest_positions[reference][0],
                self.rest_positions[index][1] - self.rest_positions[reference][1],
                self.rest_positions[index][2] - self.rest_positions[reference][2],
            ];
            let local_offset = [
                dot3(rest_delta, rest_basis[0]),
                dot3(rest_delta, rest_basis[1]),
                dot3(rest_delta, rest_basis[2]),
            ];
            let expected_offset = [
                current_basis[0][0] * local_offset[0]
                    + current_basis[1][0] * local_offset[1]
                    + current_basis[2][0] * local_offset[2],
                current_basis[0][1] * local_offset[0]
                    + current_basis[1][1] * local_offset[1]
                    + current_basis[2][1] * local_offset[2],
                current_basis[0][2] * local_offset[0]
                    + current_basis[1][2] * local_offset[1]
                    + current_basis[2][2] * local_offset[2],
            ];
            let constraint_error = [
                upper.x - mount.x - expected_offset[0],
                upper.y - mount.y - expected_offset[1],
                upper.z - mount.z - expected_offset[2],
            ];
            let inv_mass_sum = upper.inv_mass + mount.inv_mass;
            if inv_mass_sum <= 0.0 {
                continue;
            }
            let correction = [
                -constraint_error[0] / inv_mass_sum,
                -constraint_error[1] / inv_mass_sum,
                -constraint_error[2] / inv_mass_sum,
            ];
            if !mount.fixed {
                self.nodes[reference].x -= correction[0] * mount.inv_mass;
                self.nodes[reference].y -= correction[1] * mount.inv_mass;
                self.nodes[reference].z -= correction[2] * mount.inv_mass;
            }
            if !upper.fixed {
                self.nodes[index].x += correction[0] * upper.inv_mass;
                self.nodes[index].y += correction[1] * upper.inv_mass;
                self.nodes[index].z += correction[2] * upper.inv_mass;
            }
        }
    }

    // Area preservation is the soft-body equivalent of an angular/bend
    // constraint for the mesh triangles. It resists shearing while still
    // allowing beams to break and the body to crumple.
    pub(crate) fn solve_triangle_constraints(&mut self, dt2: f64) {
        const TRIANGLE_COMPLIANCE: f64 = 1e-5;
        for tri in &mut self.triangles {
            if tri.a >= self.nodes.len() || tri.b >= self.nodes.len() || tri.c >= self.nodes.len() {
                continue;
            }
            let a = self.nodes[tri.a];
            let b = self.nodes[tri.b];
            let c = self.nodes[tri.c];
            let ab = [b.x - a.x, b.y - a.y, b.z - a.z];
            let ac = [c.x - a.x, c.y - a.y, c.z - a.z];
            let area_normal = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];
            let double_area = (area_normal[0] * area_normal[0]
                + area_normal[1] * area_normal[1]
                + area_normal[2] * area_normal[2])
                .sqrt();
            if double_area < 1e-10 {
                continue;
            }
            let normal = [
                area_normal[0] / double_area,
                area_normal[1] / double_area,
                area_normal[2] / double_area,
            ];
            let cross = |left: [f64; 3], right: [f64; 3]| {
                [
                    left[1] * right[2] - left[2] * right[1],
                    left[2] * right[0] - left[0] * right[2],
                    left[0] * right[1] - left[1] * right[0],
                ]
            };
            let scale = |value: [f64; 3]| [value[0] * 0.5, value[1] * 0.5, value[2] * 0.5];
            // Exact gradients of A = 0.5 * |(b-a) x (c-a)|.
            let gradients = [
                scale(cross([b.x - c.x, b.y - c.y, b.z - c.z], normal)),
                scale(cross([c.x - a.x, c.y - a.y, c.z - a.z], normal)),
                scale(cross(normal, [b.x - a.x, b.y - a.y, b.z - a.z])),
            ];
            let area = double_area * 0.5;
            let error = area - tri.rest_area;
            let indices = [tri.a, tri.b, tri.c];
            let weighted_gradient_sum = indices
                .iter()
                .zip(gradients.iter())
                .map(|(index, gradient)| {
                    let inv_mass = self.nodes[*index].inv_mass;
                    inv_mass
                        * (gradient[0] * gradient[0]
                            + gradient[1] * gradient[1]
                            + gradient[2] * gradient[2])
                })
                .sum::<f64>();
            let alpha = TRIANGLE_COMPLIANCE / dt2.max(1e-12);
            let denominator = weighted_gradient_sum + alpha;
            if denominator <= 1e-12 {
                continue;
            }
            let delta_lambda = (-error - alpha * tri.lambda) / denominator;
            tri.lambda += delta_lambda;
            for (index, gradient) in indices.into_iter().zip(gradients) {
                let inv_mass = self.nodes[index].inv_mass;
                if inv_mass > 0.0 {
                    self.nodes[index].x += delta_lambda * inv_mass * gradient[0];
                    self.nodes[index].y += delta_lambda * inv_mass * gradient[1];
                    self.nodes[index].z += delta_lambda * inv_mass * gradient[2];
                }
            }
        }
    }
}
