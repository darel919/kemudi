//! Force accumulation, semi-implicit integration, and aerodynamic drag.

use crate::math::finite_or_zero;
use crate::types::PhysicsWorld;

impl PhysicsWorld {
    pub(crate) fn apply_forces(&mut self) {
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            // Gravity belongs to each mass node. Suspension forces then travel
            // through the authored beams into the chassis instead of
            // teleporting upper-cage weight onto the wheel mounts.
            node.fy += node.mass * self.gravity;
        }
    }

    pub(crate) fn integrate_velocities(&mut self, dt: f64) {
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            node.vx += node.fx * node.inv_mass * dt;
            node.vy += node.fy * node.inv_mass * dt;
            node.vz += node.fz * node.inv_mass * dt;
            node.last_force = (node.fx * node.fx + node.fy * node.fy + node.fz * node.fz).sqrt();
            node.fx = 0.0;
            node.fy = 0.0;
            node.fz = 0.0;
            node.vx = finite_or_zero(node.vx).clamp(-200.0, 200.0);
            node.vy = finite_or_zero(node.vy).clamp(-200.0, 200.0);
            node.vz = finite_or_zero(node.vz).clamp(-200.0, 200.0);
            node.x = finite_or_zero(node.x + node.vx * dt);
            node.y = finite_or_zero(node.y + node.vy * dt);
            node.z = finite_or_zero(node.z + node.vz * dt);
        }
    }

    pub(crate) fn apply_drag(&mut self) {
        let total_mass = self
            .nodes
            .iter()
            .filter(|node| !node.fixed)
            .map(|node| node.mass)
            .sum::<f64>()
            .max(1.0);
        let average_velocity =
            self.nodes
                .iter()
                .filter(|node| !node.fixed)
                .fold([0.0; 3], |mut average, node| {
                    average[0] += node.vx * node.mass / total_mass;
                    average[1] += node.vy * node.mass / total_mass;
                    average[2] += node.vz * node.mass / total_mass;
                    average
                });
        let speed = (average_velocity[0] * average_velocity[0]
            + average_velocity[1] * average_velocity[1]
            + average_velocity[2] * average_velocity[2])
            .sqrt();
        let total_drag =
            0.5 * self.air_density * self.drag_coefficient * self.frontal_area * speed * speed;
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            let node_speed = (node.vx * node.vx + node.vy * node.vy + node.vz * node.vz).sqrt();
            if node_speed > 1e-6 {
                // Aerodynamic drag is defined by frontal area, not vehicle
                // mass. Distribute the vehicle drag by node mass so the
                // deformable cage receives one bounded body-level force.
                let drag = total_drag * node.mass / total_mass;
                let factor =
                    (1.0 - drag * self.fixed_dt / (node.mass * node_speed + 1e-6)).max(0.0);
                node.vx *= factor;
                node.vy *= factor;
                node.vz *= factor;
            }
        }
    }
}
