//! Force accumulation, semi-implicit integration, and aerodynamic drag.

use crate::math::finite_or_zero;
use crate::types::PhysicsWorld;

/// Safe body-level aerodynamic downforce using only the existing runtime aero
/// fields. The coefficient is tied to the authored drag coefficient as a
/// conservative fallback until triangle lift data is added to the schema.
pub(crate) fn body_downforce_force(
    speed: f64,
    air_density: f64,
    drag_coefficient: f64,
    frontal_area: f64,
) -> f64 {
    let safe_speed = if speed.is_finite() { speed.abs() } else { 0.0 };
    let rho = if air_density.is_finite() {
        air_density.max(0.0)
    } else {
        0.0
    };
    let area = if frontal_area.is_finite() {
        frontal_area.max(0.0)
    } else {
        0.0
    };
    let drag = if drag_coefficient.is_finite() {
        drag_coefficient.abs()
    } else {
        0.0
    };
    let downforce_coefficient = (drag * 0.25).clamp(0.0, 1.0);
    (0.5 * rho * area * downforce_coefficient * safe_speed * safe_speed).max(0.0)
}

impl PhysicsWorld {
    /// Distribute fuel mass across dynamic nodes as a deterministic tank
    /// approximation. This keeps fuel burn/refueling coupled to gravity,
    /// suspension load, and chassis inertia without inventing a tank location
    /// that is not present in the vehicle schema.
    pub(crate) fn update_mass_properties(&mut self) {
        let dynamic_count = self.nodes.iter().filter(|node| !node.fixed).count();
        if dynamic_count == 0 {
            return;
        }
        let fuel_mass = (self.fuel.current_level * self.fuel.fuel_density)
            .is_finite()
            .then_some(self.fuel.current_level * self.fuel.fuel_density)
            .unwrap_or(0.0)
            .max(0.0);
        let fuel_share = fuel_mass / dynamic_count as f64;
        for (index, node) in self.nodes.iter_mut().enumerate() {
            if node.fixed {
                continue;
            }
            let base_mass = self
                .base_node_masses
                .get(index)
                .copied()
                .unwrap_or(node.mass)
                .max(1e-6);
            node.mass = base_mass + fuel_share;
            node.inv_mass = 1.0 / node.mass;
        }
    }

    pub(crate) fn apply_forces(&mut self) {
        let total_mass = self
            .nodes
            .iter()
            .filter(|node| !node.fixed)
            .map(|node| node.mass.max(0.0))
            .sum::<f64>();
        if total_mass <= 0.0 || !total_mass.is_finite() {
            return;
        }
        let average_velocity =
            self.nodes
                .iter()
                .filter(|node| !node.fixed)
                .fold([0.0; 3], |mut average, node| {
                    let mass = node.mass.max(0.0);
                    average[0] += node.vx * mass / total_mass;
                    average[1] += node.vy * mass / total_mass;
                    average[2] += node.vz * mass / total_mass;
                    average
                });
        let speed = (average_velocity[0] * average_velocity[0]
            + average_velocity[1] * average_velocity[1]
            + average_velocity[2] * average_velocity[2])
            .sqrt();
        let downforce = body_downforce_force(
            speed,
            self.air_density,
            self.drag_coefficient,
            self.frontal_area,
        );
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            // Gravity belongs to each mass node. Suspension forces then travel
            // through the authored beams into the chassis instead of
            // teleporting upper-cage weight onto the wheel mounts.
            node.fy += node.mass * self.gravity;
            // The body-level aero term is deliberately downward-only and is
            // distributed by mass so it remains connected to the current node
            // runtime without requiring speculative triangle schema fields.
            node.fy -= downforce * node.mass.max(0.0) / total_mass;
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
