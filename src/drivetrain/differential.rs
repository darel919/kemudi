use serde::{Deserialize, Serialize};

// === Differential ===

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum DiffMode {
    Open,
    Locked,
    LimitedSlip,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Differential {
    pub mode: DiffMode,
    pub bias: f64,
    pub left_ratio: f64,
    pub right_ratio: f64,
}

impl Default for Differential {
    fn default() -> Self {
        Self {
            mode: DiffMode::Open,
            bias: 0.5,
            left_ratio: 0.5,
            right_ratio: 0.5,
        }
    }
}

impl Differential {
    pub fn apply_differential(
        &mut self,
        torque: f64,
        left_grip: f64,
        right_grip: f64,
    ) -> (f64, f64) {
        match self.mode {
            DiffMode::Locked => {
                self.left_ratio = 0.5;
                self.right_ratio = 0.5;
                (torque * 0.5, torque * 0.5)
            }
            DiffMode::Open => {
                self.left_ratio = 0.5;
                self.right_ratio = 0.5;
                (torque * 0.5, torque * 0.5)
            }
            DiffMode::LimitedSlip => {
                let total_grip = left_grip + right_grip + 1e-6;
                let base = torque * self.bias;
                let slip_torque = torque * (1.0 - self.bias);
                let left = base + slip_torque * (left_grip / total_grip);
                let right = base + slip_torque * (right_grip / total_grip);
                self.left_ratio = left / (torque + 1e-6);
                self.right_ratio = right / (torque + 1e-6);
                (left, right)
            }
        }
    }
}
