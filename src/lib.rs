use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

#[wasm_bindgen]
#[derive(Debug, Clone)]
pub struct PhysicsWorld {
    vehicles: Vec<VehicleState>,
    gravity: f64,
    time: f64,
}

#[wasm_bindgen]
impl PhysicsWorld {
    #[wasm_bindgen(constructor)]
    pub fn new() -> PhysicsWorld {
        console_error_panic_hook::set_once();
        PhysicsWorld {
            vehicles: Vec::new(),
            gravity: -9.81,
            time: 0.0,
        }
    }

    pub fn step(&mut self, dt: f64) {
        let dt = dt.max(0.0).min(1.0 / 20.0); // clamp to avoid tunneling
        self.time += dt;
        for v in &mut self.vehicles {
            // simple gravity integration
            v.vy += self.gravity * dt;
            v.x += v.vx * dt;
            v.y += v.vy * dt;
            v.z += v.vz * dt;
            // ground plane
            if v.y < 0.0 {
                v.y = 0.0;
                v.vy = 0.0;
            }
        }
    }

    pub fn load_vehicle(&mut self, x: f64, y: f64, z: f64) -> usize {
        self.vehicles.push(VehicleState {
            x, y, z,
            vx: 0.0, vy: 0.0, vz: 0.0,
        });
        self.vehicles.len() - 1
    }

    pub fn get_positions(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.vehicles.len() * 3);
        for v in &self.vehicles {
            out.push(v.x);
            out.push(v.y);
            out.push(v.z);
        }
        out
    }

    pub fn apply_force(&mut self, index: usize, fx: f64, fy: f64, fz: f64) {
        if let Some(v) = self.vehicles.get_mut(index) {
            v.vx += fx;
            v.vy += fy;
            v.vz += fz;
        }
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}
