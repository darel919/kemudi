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
    /// Preserve the original public helper for callers that do not track
    /// individual output speeds. The simulation uses
    /// `apply_differential_with_speeds` so a locked axle can react to a
    /// left/right speed mismatch.
    pub fn apply_differential(
        &mut self,
        torque: f64,
        left_grip: f64,
        right_grip: f64,
    ) -> (f64, f64) {
        self.apply_differential_with_speeds(torque, left_grip, right_grip, 0.0, 0.0)
    }

    pub fn apply_differential_with_speeds(
        &mut self,
        torque: f64,
        left_grip: f64,
        right_grip: f64,
        left_speed: f64,
        right_speed: f64,
    ) -> (f64, f64) {
        let safe_torque = if torque.is_finite() { torque } else { 0.0 };
        let left_grip = finite_non_negative(left_grip);
        let right_grip = finite_non_negative(right_grip);
        let grip_total = left_grip + right_grip;
        let grip_share = if grip_total > 1e-9 {
            left_grip / grip_total
        } else {
            0.5
        };

        let left_ratio = match self.mode {
            DiffMode::Open => 0.5,
            DiffMode::Locked => {
                // A locked axle can route reaction torque through the wheel
                // that still has traction. It also resists output-speed
                // mismatch: when the left output is faster, transfer torque
                // toward the right output (and vice versa). This is a
                // bounded approximation of the lock reaction because wheel
                // shaft compliance is not yet represented explicitly.
                let left_speed = finite_or_zero(left_speed);
                let right_speed = finite_or_zero(right_speed);
                let speed_scale = left_speed.abs().max(right_speed.abs()).max(1.0);
                let speed_transfer = ((right_speed - left_speed) / speed_scale) * 0.25;
                (grip_share + speed_transfer).clamp(0.0, 1.0)
            }
            DiffMode::LimitedSlip => {
                // `bias` is the fraction of the open split that may be moved
                // toward available traction. This keeps output torque
                // conserved for every calibration value.
                let locking = finite_non_negative(self.bias).clamp(0.0, 1.0);
                (0.5 * (1.0 - locking) + grip_share * locking).clamp(0.0, 1.0)
            }
        };
        self.left_ratio = left_ratio;
        self.right_ratio = 1.0 - left_ratio;
        (
            safe_torque * self.left_ratio,
            safe_torque * self.right_ratio,
        )
    }
}

fn finite_non_negative(value: f64) -> f64 {
    if value.is_finite() {
        value.max(0.0)
    } else {
        0.0
    }
}

fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}
