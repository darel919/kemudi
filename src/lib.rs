use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

pub mod drivetrain;
pub mod engine;
pub mod safety;
pub mod suspension;
pub mod terrain_contact;
pub mod tires;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Node {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub mass: f64,
    pub inv_mass: f64,
    pub fixed: bool,
    pub collision: bool,
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    pub last_force: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Beam {
    pub id: usize,
    pub node_a: usize,
    pub node_b: usize,
    pub stiffness: f64,
    pub damping: f64,
    pub strength: f64,
    pub length: f64,
    pub broken: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub nodes: Vec<Node>,
    pub beams: Vec<Beam>,
    pub triangles: Vec<[usize; 3]>,
}

#[wasm_bindgen]
pub struct PhysicsWorld {
    nodes: Vec<Node>,
    beams: Vec<Beam>,
    gravity: f64,
    time: f64,
    ground_y: f64,
    air_density: f64,
    drag_coefficient: f64,
    accumulator: f64,
    fixed_dt: f64,
    max_substeps: u32,
}

#[wasm_bindgen]
impl PhysicsWorld {
    #[wasm_bindgen(constructor)]
    pub fn new() -> PhysicsWorld {
        PhysicsWorld {
            nodes: Vec::new(),
            beams: Vec::new(),
            gravity: -9.81,
            time: 0.0,
            ground_y: 0.0,
            air_density: 1.225,
            drag_coefficient: 0.5,
            accumulator: 0.0,
            fixed_dt: 1.0 / 60.0,
            max_substeps: 8,
        }
    }

    pub fn add_node(&mut self, id: usize, x: f64, y: f64, z: f64, mass: f64, fixed: bool) {
        let safe_mass = if mass.is_finite() && mass > 0.0 {
            mass
        } else {
            1.0
        };
        let inv_mass = if fixed { 0.0 } else { 1.0 / safe_mass };
        self.nodes.push(Node {
            id,
            x: if x.is_finite() { x } else { 0.0 },
            y: if y.is_finite() { y } else { 0.0 },
            z: if z.is_finite() { z } else { 0.0 },
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            mass: safe_mass,
            inv_mass,
            fixed,
            collision: !fixed,
            fx: 0.0,
            fy: 0.0,
            fz: 0.0,
            last_force: 0.0,
        });
    }

    pub fn add_beam(
        &mut self,
        id: usize,
        node_a: usize,
        node_b: usize,
        stiffness: f64,
        damping: f64,
        strength: f64,
    ) {
        let length = if let (Some(a), Some(b)) = (self.nodes.get(node_a), self.nodes.get(node_b)) {
            ((b.x - a.x).powi(2) + (b.y - a.y).powi(2) + (b.z - a.z).powi(2)).sqrt()
        } else {
            1.0
        };
        self.beams.push(Beam {
            id,
            node_a,
            node_b,
            stiffness,
            damping,
            strength,
            length,
            broken: false,
        });
    }

    pub fn add_vehicle(&mut self, _x: f64, _y: f64, _z: f64) -> usize {
        let vehicle_id = self.nodes.len();
        // Vehicle data is loaded via load_vehicle_data which sets nodes/beams externally
        vehicle_id
    }

    pub fn step(&mut self, dt: f64) {
        let frame_dt = if dt.is_finite() {
            dt.clamp(0.0, 0.25)
        } else {
            0.0
        };
        self.accumulator += frame_dt;

        let mut substeps = 0;
        while self.accumulator + 1e-12 >= self.fixed_dt && substeps < self.max_substeps {
            self.step_fixed();
            self.accumulator -= self.fixed_dt;
            self.time += self.fixed_dt;
            substeps += 1;
        }

        // Drop excess catch-up time after a long stall instead of allowing an
        // unbounded backlog to monopolize the worker.
        if substeps == self.max_substeps {
            self.accumulator = self.accumulator.min(self.fixed_dt);
        }
    }

    pub fn get_positions_flat(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.nodes.len() * 3);
        for node in &self.nodes {
            out.push(node.x);
            out.push(node.y);
            out.push(node.z);
        }
        out
    }

    pub fn get_velocities_flat(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.nodes.len() * 3);
        for node in &self.nodes {
            out.push(node.vx);
            out.push(node.vy);
            out.push(node.vz);
        }
        out
    }

    pub fn apply_force(&mut self, node_id: usize, fx: f64, fy: f64, fz: f64) {
        if let Some(node) = self.nodes.get_mut(node_id) {
            if !node.fixed {
                if fx.is_finite() {
                    node.fx += fx;
                }
                if fy.is_finite() {
                    node.fy += fy;
                }
                if fz.is_finite() {
                    node.fz += fz;
                }
            }
        }
    }

    pub fn get_node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn get_beam_count(&self) -> usize {
        self.beams.len()
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }
}

impl PhysicsWorld {
    fn step_fixed(&mut self) {
        self.apply_forces();
        self.integrate_velocities(self.fixed_dt);
        self.solve_beam_constraints();
        self.integrate_positions(self.fixed_dt);
        self.collide_with_ground();
        self.apply_drag();
    }

    fn apply_forces(&mut self) {
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            // Gravity and external forces are accumulated as force, then
            // consumed exactly once by the semi-implicit Euler integration.
            node.fy += node.mass * self.gravity;
        }
    }

    fn integrate_velocities(&mut self, dt: f64) {
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

            if !node.vx.is_finite() {
                node.vx = 0.0;
            }
            if !node.vy.is_finite() {
                node.vy = 0.0;
            }
            if !node.vz.is_finite() {
                node.vz = 0.0;
            }
            node.x += node.vx * dt;
            node.y += node.vy * dt;
            node.z += node.vz * dt;

            if !node.x.is_finite() {
                node.x = 0.0;
            }
            if !node.y.is_finite() {
                node.y = self.ground_y;
            }
            if !node.z.is_finite() {
                node.z = 0.0;
            }
        }
    }

    fn solve_beam_constraints(&mut self) {
        let iterations = 4;
        for _ in 0..iterations {
            // Snapshot positions for constraint solving
            let node_count = self.nodes.len();
            let mut pos: Vec<[f64; 3]> = Vec::with_capacity(node_count);
            for n in &self.nodes {
                pos.push([n.x, n.y, n.z]);
            }

            for beam in &mut self.beams {
                if beam.broken {
                    continue;
                }
                let a = beam.node_a;
                let b = beam.node_b;
                if a >= node_count || b >= node_count {
                    continue;
                }

                let dx = pos[b][0] - pos[a][0];
                let dy = pos[b][1] - pos[a][1];
                let dz = pos[b][2] - pos[a][2];
                let current_length = (dx * dx + dy * dy + dz * dz).sqrt();

                if current_length < 1e-10 {
                    continue;
                }

                let error = current_length - beam.length;
                if error.abs() < 1e-4 {
                    continue;
                }

                let inv_a = self.nodes[a].inv_mass;
                let inv_b = self.nodes[b].inv_mass;
                let inv_sum = inv_a + inv_b;
                if inv_sum <= 0.0 {
                    continue;
                }

                let stiffness = beam.stiffness;
                let lambda = error / inv_sum / (stiffness + 0.01);

                let nx = dx / current_length;
                let ny = dy / current_length;
                let nz = dz / current_length;

                // PBD correction: Δp_a = +λ * w_a * n, Δp_b = -λ * w_b * n
                // n points from a to b
                if !self.nodes[a].fixed {
                    self.nodes[a].x += lambda * inv_a * nx;
                    self.nodes[a].y += lambda * inv_a * ny;
                    self.nodes[a].z += lambda * inv_a * nz;
                }
                if !self.nodes[b].fixed {
                    self.nodes[b].x -= lambda * inv_b * nx;
                    self.nodes[b].y -= lambda * inv_b * ny;
                    self.nodes[b].z -= lambda * inv_b * nz;
                }

                // Velocity damping from constraint error
                let damp = beam.damping * error;
                if !self.nodes[a].fixed {
                    self.nodes[a].vx += damp * inv_a * nx;
                    self.nodes[a].vy += damp * inv_a * ny;
                    self.nodes[a].vz += damp * inv_a * nz;
                }
                if !self.nodes[b].fixed {
                    self.nodes[b].vx -= damp * inv_b * nx;
                    self.nodes[b].vy -= damp * inv_b * ny;
                    self.nodes[b].vz -= damp * inv_b * nz;
                }

                let applied_load = (self.nodes[a].last_force + self.nodes[b].last_force) * 0.5;
                let spring_load = error.abs() * beam.stiffness;
                if applied_load.max(spring_load) > beam.strength {
                    beam.broken = true;
                }
            }
        }
    }

    fn integrate_positions(&mut self, _dt: f64) {
        // Positions already updated in integrate_velocities (semi-implicit Euler)
    }

    fn collide_with_ground(&mut self) {
        for node in &mut self.nodes {
            if !node.collision {
                continue;
            }
            if node.y < self.ground_y {
                node.y = self.ground_y;
                node.vy = (node.vy * -0.3).abs(); // restitution
            }
        }
    }

    fn apply_drag(&mut self) {
        for node in &mut self.nodes {
            if node.fixed {
                continue;
            }
            let speed = (node.vx * node.vx + node.vy * node.vy + node.vz * node.vz).sqrt();
            if speed < 1e-6 {
                continue;
            }
            let drag = 0.5 * self.air_density * self.drag_coefficient * node.mass * speed * speed;
            let factor = 1.0 - (drag / (node.mass * speed + 1e-6));
            let factor = factor.max(0.9); // cap drag so nodes don't reverse
            node.vx *= factor;
            node.vy *= factor;
            node.vz *= factor;
        }
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

// ===== Tests =====

#[cfg(test)]
mod tests {
    use super::*;

    fn create_world_with_two_nodes() -> PhysicsWorld {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 0.0, 0.0, 1.0, true);
        w.add_node(1, 0.0, 3.0, 0.0, 1.0, false);
        w.add_beam(0, 0, 1, 1000.0, 0.5, 1000.0);
        w
    }

    #[test]
    fn test_node_creation() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 1.0, 2.0, 3.0, 5.0, false);
        assert_eq!(w.nodes.len(), 1);
        let n = &w.nodes[0];
        assert_eq!(n.id, 0);
        assert!((n.x - 1.0).abs() < 1e-10);
        assert!((n.mass - 5.0).abs() < 1e-10);
        assert!((n.inv_mass - 0.2).abs() < 1e-10);
        assert!(!n.fixed);
        assert!(n.collision);
    }

    #[test]
    fn test_fixed_node() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 5.0, 0.0, 1.0, true);
        assert!(w.nodes[0].fixed);
        assert!(!w.nodes[0].collision);
        assert!((w.nodes[0].inv_mass).abs() < 1e-10);
    }

    #[test]
    fn test_beam_rest_length() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 0.0, 0.0, 1.0, true);
        w.add_node(1, 0.0, 0.0, 3.0, 1.0, false);
        w.add_beam(0, 0, 1, 1000.0, 0.5, 1000.0);
        // Rest length should be computed from initial distance
        assert!((w.beams[0].length - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_gravity_pulls_down() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 10.0, 0.0, 1.0, false);
        w.step(1.0 / 60.0);
        assert!(w.nodes[0].y < 10.0, "Node should move downward");
        assert!(w.nodes[0].vy < 0.0, "Velocity should be negative");
    }

    #[test]
    fn test_fixed_timestep_caps_catch_up_work() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 10.0, 0.0, 1.0, false);

        w.step(0.005);
        assert_eq!(w.get_time(), 0.0, "partial frame should remain accumulated");
        w.step(0.02);
        assert!((w.get_time() - 1.0 / 60.0).abs() < 1e-9);

        let before = w.get_time();
        w.step(1.0);
        assert!(w.get_time() - before <= 8.0 / 60.0 + 1e-9);
    }

    #[test]
    fn test_fixed_nodes_dont_move() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 5.0, 0.0, 1.0, true);
        for _ in 0..120 {
            w.step(1.0 / 60.0);
        }
        assert!(
            (w.nodes[0].y - 5.0).abs() < 1e-10,
            "Fixed node must not move"
        );
    }

    #[test]
    fn test_ground_collision() {
        let mut w = PhysicsWorld::new();
        w.ground_y = 0.0;
        w.add_node(0, 0.0, 0.5, 0.0, 1.0, false);

        for _ in 0..300 {
            w.step(1.0 / 60.0);
        }
        assert!(
            w.nodes[0].y >= -0.01,
            "Node should not go below ground, got {}",
            w.nodes[0].y
        );
    }

    #[test]
    fn test_beam_constraint_maintains_length() {
        let mut w = create_world_with_two_nodes();

        for _ in 0..120 {
            w.step(1.0 / 60.0);
        }

        let dx = w.nodes[1].x - w.nodes[0].x;
        let dy = w.nodes[1].y - w.nodes[0].y;
        let dz = w.nodes[1].z - w.nodes[0].z;
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(
            dist > 0.5 && dist < 5.0,
            "Beam should maintain reasonable separation, got {}",
            dist
        );
    }

    #[test]
    fn test_beam_breaking() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 0.0, 0.0, 1.0, true);
        w.add_node(1, 1.0, 0.0, 0.0, 1.0, false);
        // Very weak beam
        w.add_beam(0, 0, 1, 1.0, 0.0, 0.05);

        w.apply_force(1, 100.0, 0.0, 0.0);
        w.step(1.0 / 60.0);
        assert!(
            w.beams[0].broken,
            "Weak beam should break under large force"
        );
    }

    #[test]
    fn test_get_positions_flat() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 1.0, 2.0, 3.0, 1.0, false);
        w.add_node(1, 4.0, 5.0, 6.0, 1.0, true);
        let pos = w.get_positions_flat();
        assert_eq!(pos.len(), 6);
        assert!((pos[0] - 1.0).abs() < 1e-10);
        assert!((pos[3] - 4.0).abs() < 1e-10);
        assert!((pos[4] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_apply_force() {
        let mut w = PhysicsWorld::new();
        w.add_node(0, 0.0, 0.0, 0.0, 1.0, false);
        w.step(1.0 / 60.0);
        let vy_before = w.nodes[0].vy;
        w.apply_force(0, 0.0, 50.0, 0.0);
        w.step(1.0 / 60.0);
        // After applying upward force, vy should be higher than without
        assert!(
            w.nodes[0].vy > vy_before,
            "Applied force should increase velocity"
        );
    }

    #[test]
    fn test_multiple_nodes_and_beams() {
        let mut w = PhysicsWorld::new();
        // 4 nodes in a square
        w.add_node(0, -1.0, 0.5, -1.0, 1.0, false);
        w.add_node(1, 1.0, 0.5, -1.0, 1.0, false);
        w.add_node(2, -1.0, 0.5, 1.0, 1.0, false);
        w.add_node(3, 1.0, 0.5, 1.0, 1.0, false);

        // Connect edges
        w.add_beam(0, 0, 1, 1000.0, 0.5, 1000.0);
        w.add_beam(1, 2, 3, 1000.0, 0.5, 1000.0);
        w.add_beam(2, 0, 2, 1000.0, 0.5, 1000.0);
        w.add_beam(3, 1, 3, 1000.0, 0.5, 1000.0);

        for _ in 0..60 {
            w.step(1.0 / 60.0);
        }

        assert_eq!(w.get_node_count(), 4);
        assert_eq!(w.get_beam_count(), 4);
    }
}
