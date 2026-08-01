use serde::{Deserialize, Serialize};

/// Engine RPM range and torque curve
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Engine {
    pub rpm: f64,
    pub idle_rpm: f64,
    pub redline_rpm: f64,
    pub rev_limiter_rpm: f64,
    /// Torque curve: [(rpm, torque_nm), ...] sorted ascending by rpm
    pub torque_curve: Vec<(f64, f64)>,
    pub throttle_response: f64,
    pub engine_braking: f64,
}

impl Default for Engine {
    fn default() -> Self {
        Self {
            rpm: 800.0,
            idle_rpm: 800.0,
            redline_rpm: 7000.0,
            rev_limiter_rpm: 7500.0,
            torque_curve: vec![
                (0.0, 100.0),
                (1000.0, 150.0),
                (2000.0, 200.0),
                (3000.0, 250.0),
                (4000.0, 280.0),
                (5000.0, 260.0),
                (6000.0, 220.0),
                (7000.0, 160.0),
                (8000.0, 100.0),
            ],
            throttle_response: 0.8,
            engine_braking: 30.0,
        }
    }
}

impl Engine {
    pub fn torque_at_rpm(&self, rpm: f64) -> f64 {
        if self.torque_curve.is_empty() {
            return 0.0;
        }
        if rpm <= self.torque_curve[0].0 {
            return self.torque_curve[0].1;
        }
        if rpm >= self.torque_curve.last().unwrap().0 {
            return self.torque_curve.last().unwrap().1;
        }
        for w in self.torque_curve.windows(2) {
            if rpm >= w[0].0 && rpm <= w[1].0 {
                let t = (rpm - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + t * (w[1].1 - w[0].1);
            }
        }
        0.0
    }

    pub fn update(&mut self, throttle: f64, dt: f64) {
        let target_rpm = if throttle > 0.01 {
            self.idle_rpm + (self.redline_rpm - self.idle_rpm) * throttle
        } else {
            self.idle_rpm
        };
        let rate = self.throttle_response * 10.0;
        self.rpm += (target_rpm - self.rpm) * rate * dt;
        self.rpm = self.rpm.clamp(0.0, self.rev_limiter_rpm);
    }
}
