use super::{
    AutoShiftLogic, ClutchShock, Differential, Engine, OverrevCause, ShiftPhase, Transmission,
    TransmissionMode,
};

// === Combined Drivetrain ===

pub struct Drivetrain {
    pub engine: Engine,
    pub transmission: Transmission,
    /// Driven-wheel angular velocity in radians per second. Chassis node
    /// velocity is the only authoritative vehicle linear velocity.
    pub wheel_speed: f64,
    pub throttle_input: f64,
    pub brake_input: f64,
    pub clutch_input: f64,
    pub shift_phase: ShiftPhase,
    pub shift_timer: f64,
    pub shift_duration: f64,
    /// Multiplier supplied by the TCM actuator model for the next shift.
    pub shift_duration_multiplier: f64,
    pub pending_gear: i32,
    pub auto_shift: AutoShiftLogic,
    pub differential: Differential,
    pub time: f64,
    pub is_stalled: bool,
    /// Drivetrain-forced over-rev events (not rev-limiter limited).
    pub overrev_events: Vec<OverrevCause>,
    /// Current clutch shock state.
    pub clutch_shock: ClutchShock,
    /// Maximum allowed downshift RPM (beyond this, shift is blocked or causes damage).
    pub max_downshift_rpm: f64,
    /// Last propulsion torque delivered to the driven wheels after engine
    /// braking. Service-brake torque is intentionally excluded because the
    /// chassis solver applies braking at each grounded contact patch.
    pub last_drive_torque: f64,
    /// Automatic transmission torque-converter coupling (0-1). Manual
    /// transmissions keep this at 1.0 and use clutch engagement instead.
    pub converter_coupling: f64,
    /// Torque multiplication applied by the unlocked converter.
    pub converter_torque_multiplier: f64,
    /// Effective driven tire radius used to couple wheel angular speed to
    /// chassis speed. Configured from the vehicle definition at runtime.
    pub wheel_radius: f64,
    /// Configured rotational inertia of the driven wheel set. Zero retains
    /// the legacy vehicle-mass fallback for direct drivetrain callers.
    pub wheel_inertia: f64,
}

impl Drivetrain {
    pub fn new() -> Drivetrain {
        Drivetrain {
            engine: Engine::default(),
            transmission: Transmission::default(),
            wheel_speed: 0.0,
            throttle_input: 0.0,
            brake_input: 0.0,
            clutch_input: 0.0,
            shift_phase: ShiftPhase::Idle,
            shift_timer: 0.0,
            shift_duration: 0.15,
            shift_duration_multiplier: 1.0,
            pending_gear: 0,
            auto_shift: AutoShiftLogic::default(),
            differential: Differential::default(),
            time: 0.0,
            is_stalled: false,
            overrev_events: Vec::new(),
            clutch_shock: ClutchShock::default(),
            max_downshift_rpm: 8000.0,
            last_drive_torque: 0.0,
            converter_coupling: 1.0,
            converter_torque_multiplier: 1.0,
            wheel_radius: 0.3,
            wheel_inertia: 0.0,
        }
    }

    pub fn update(&mut self, throttle: f64, brake: f64, clutch: f64, dt: f64, vehicle_mass: f64) {
        self.time += dt;
        self.throttle_input = throttle.clamp(0.0, 1.0);
        self.brake_input = brake.clamp(0.0, 1.0);
        self.clutch_input = clutch.clamp(0.0, 1.0);

        // Update shift phase
        self.update_shift_phase(dt);

        // Clutch engagement
        self.transmission.clutch_engagement = if self.shift_phase == ShiftPhase::Neutral {
            0.0
        } else if self.transmission.mode == TransmissionMode::Automatic {
            // An automatic's launch device is the torque converter. Its
            // coupling is modeled separately from the mechanical gear clutch.
            1.0
        } else {
            1.0 - self.clutch_input
        };

        // A fully engaged manual clutch is a mechanical speed connection,
        // not a soft RPM synchronizer. Keeping it on the old first-order
        // blend lets the chassis travel at a high road speed while the
        // engine remains near idle, so first gear behaves like a CVT until
        // the blend catches up. Automatic transmissions only get this rigid
        // connection when the torque converter reports lock-up.
        let rigid_coupling = self.transmission.clutch_engagement > 0.95
            && (self.transmission.mode == TransmissionMode::Manual
                || self.converter_coupling > 0.95)
            && (self.wheel_speed.abs() > 0.5 || self.engine.rpm >= self.engine.idle_rpm * 0.7);

        // Engine torque — throttle gates power delivery
        let coupled_rpm = self
            .transmission
            .engine_rpm_from_wheel_speed(self.wheel_speed);
        let engine_torque = if rigid_coupling && coupled_rpm >= self.engine.rev_limiter_rpm {
            0.0
        } else {
            self.engine.torque_at_rpm(self.engine.rpm) * self.throttle_input
        };

        // Wheel torque through drivetrain
        let converter_factor = if self.transmission.mode == TransmissionMode::Automatic {
            (self.converter_coupling * self.converter_torque_multiplier).clamp(0.0, 1.2)
        } else {
            1.0
        };
        let drive_torque = self.transmission.wheel_torque(engine_torque) * converter_factor;

        // The lumped driven-wheel inertia is used by every shaft torque. A
        // resistive torque may remove at most the angular momentum available
        // in this step; otherwise explicit integration crosses zero and turns
        // engine/service braking into an alternating energy source.
        let wheel_inertia = self.effective_wheel_inertia(vehicle_mass);
        let stopping_torque = self.wheel_speed.abs() * wheel_inertia / dt.max(1e-6);

        // Engine braking
        let requested_engine_brake = if self.throttle_input < 0.05
            && self.wheel_speed.abs() > 1e-6
            && self.transmission.clutch_engagement > 0.05
        {
            self.engine.engine_braking * self.transmission.total_ratio().abs()
        } else {
            0.0
        };

        // Brake torque
        let direction = if self.wheel_speed.abs() > 1e-6 {
            self.wheel_speed.signum()
        } else {
            1.0
        };
        let requested_brake = self.brake_input * 500.0;
        let resistive_torque = (requested_engine_brake + requested_brake).min(stopping_torque);
        let engine_brake_share = if requested_engine_brake + requested_brake > 1e-9 {
            resistive_torque * requested_engine_brake / (requested_engine_brake + requested_brake)
        } else {
            0.0
        };
        let engine_brake_torque = engine_brake_share * direction;
        let brake_torque = (resistive_torque - engine_brake_share) * direction;

        // Propulsion/engine-braking torque crosses the tire contact boundary.
        // Service braking has its own per-wheel chassis force path, so keeping
        // it out of `last_drive_torque` prevents applying the same brake input
        // once through the differential and again through the contact patch.
        self.last_drive_torque = drive_torque - engine_brake_torque;
        let shaft_torque = self.last_drive_torque - brake_torque;

        let angular_accel = shaft_torque / (wheel_inertia + 1e-6);

        self.wheel_speed += angular_accel * dt;
        self.wheel_speed = self.wheel_speed.clamp(-200.0, 200.0);

        // Update engine RPM
        if self.shift_phase == ShiftPhase::Neutral {
            // Rev toward throttle target without drivetrain coupling
            self.engine.update(self.throttle_input, dt);
        } else if rigid_coupling {
            // With the clutch locked, the gear ratio is authoritative. A
            // fuel-cut limiter removes propulsion above redline, but never
            // allows the engine telemetry to lag far behind road speed.
            self.engine.rpm = coupled_rpm
                .max(self.engine.idle_rpm)
                .min(self.engine.rev_limiter_rpm);
        } else if self.transmission.clutch_engagement > 0.5
            && (self.transmission.mode != TransmissionMode::Automatic
                || self.converter_coupling > 0.95)
        {
            // Clutch engaged: engine RPM follows wheel speed through gear ratio
            let target_rpm = self
                .transmission
                .engine_rpm_from_wheel_speed(self.wheel_speed);
            let blend = self.transmission.clutch_engagement
                * self.converter_coupling
                * self.engine.throttle_response;
            self.engine.rpm +=
                (target_rpm.max(self.engine.idle_rpm) - self.engine.rpm) * blend * dt;
            // Rev limiter
            self.engine.rpm = self.engine.rpm.min(self.engine.rev_limiter_rpm);
        } else if self.transmission.mode == TransmissionMode::Automatic {
            // With the converter unlocked the engine can flare toward the
            // throttle target while only part of its speed is pulled toward
            // the turbine speed. This is the launch/slip behavior that an
            // automatic must have; treating it as a fully locked clutch makes
            // the engine stall at idle whenever the car is stationary.
            self.engine.update(self.throttle_input, dt);
            let target_rpm = self
                .transmission
                .engine_rpm_from_wheel_speed(self.wheel_speed);
            self.engine.rpm += (target_rpm.max(self.engine.idle_rpm) - self.engine.rpm)
                * self.converter_coupling
                * 4.0
                * dt;
            self.engine.rpm = self.engine.rpm.clamp(0.0, self.engine.rev_limiter_rpm);
        } else {
            self.engine.update(self.throttle_input, dt);
        }

        // Idle behavior
        if self.throttle_input < 0.05 && self.transmission.clutch_engagement > 0.5 {
            if self.engine.rpm > self.engine.idle_rpm + 100.0 {
                self.engine.rpm -= self.engine.engine_braking * dt;
            }
        }

        // Stall detection
        self.is_stalled = self.transmission.clutch_engagement > 0.5
            && self.engine.rpm < self.engine.idle_rpm * 0.7
            && self.wheel_speed.abs() < 0.5;
        if self.is_stalled {
            self.engine.rpm = self.engine.idle_rpm;
        }

        // Rev limiter
        if self.engine.rpm >= self.engine.rev_limiter_rpm {
            self.engine.rpm = self.engine.rev_limiter_rpm;
        }
    }

    fn update_shift_phase(&mut self, dt: f64) {
        match self.shift_phase {
            ShiftPhase::Idle => {}
            ShiftPhase::Disengaging => {
                self.shift_timer -= dt;
                if self.shift_timer <= 0.0 {
                    // Apply gear change
                    self.transmission.current_gear = self.pending_gear;
                    self.shift_phase = ShiftPhase::Neutral;
                    self.shift_timer = self.shift_duration / 3.0;

                    // Rev match: adjust RPM toward target for new gear
                    let target_rpm = self
                        .transmission
                        .engine_rpm_from_wheel_speed(self.wheel_speed);
                    self.engine.rpm = self.engine.rpm * 0.7 + target_rpm * 0.3;
                }
            }
            ShiftPhase::Neutral => {
                self.shift_timer -= dt;
                if self.shift_timer <= 0.0 {
                    self.shift_phase = ShiftPhase::Engaging;
                    self.shift_timer = self.shift_duration / 3.0;
                }
            }
            ShiftPhase::Engaging => {
                self.shift_timer -= dt;
                self.compute_clutch_shock(dt);
                if self.shift_timer <= 0.0 {
                    self.shift_phase = ShiftPhase::Idle;
                    self.auto_shift.record_shift(self.time);
                }
            }
        }
    }

    pub fn request_shift_up(&mut self) -> bool {
        if self.shift_phase != ShiftPhase::Idle {
            return false; // blocked
        }
        let max_gear = self.transmission.gear_ratios.len() as i32;
        if self.transmission.current_gear >= max_gear {
            return false;
        }
        self.pending_gear = self.transmission.current_gear + 1;
        self.shift_phase = ShiftPhase::Disengaging;
        self.shift_timer = self.shift_duration * self.shift_duration_multiplier / 3.0;
        true
    }

    pub fn request_shift_down(&mut self) -> bool {
        if self.shift_phase != ShiftPhase::Idle {
            return false;
        }
        if self.transmission.current_gear <= -1 {
            return false;
        }
        self.pending_gear = self.transmission.current_gear - 1;
        if self.pending_gear < 0 && self.wheel_speed.abs() > 0.5 {
            self.pending_gear = self.transmission.current_gear;
            return false;
        }
        self.shift_phase = ShiftPhase::Disengaging;
        self.shift_timer = self.shift_duration * self.shift_duration_multiplier / 3.0;
        true
    }

    /// Predict what engine RPM would result from downshifting to `target_gear`
    /// at current wheel speed. Used to check for over-rev before applying shift.
    pub fn predict_downshift_rpm(&self, wheel_speed: f64, target_gear: i32) -> f64 {
        if target_gear <= 0 {
            return self.engine.idle_rpm;
        }
        let idx = (target_gear as usize).saturating_sub(1);
        if idx >= self.transmission.gear_ratios.len() {
            return self.engine.rev_limiter_rpm;
        }
        let ratio = self.transmission.gear_ratios[idx].abs() * self.transmission.final_drive;
        if ratio < 1e-6 {
            return 0.0;
        }
        wheel_speed.abs() * ratio * 60.0 / (2.0 * std::f64::consts::PI)
    }

    /// Check if a downshift would cause over-rev, and record the cause.
    /// Returns the predicted RPM and whether the shift should be blocked.
    pub fn check_downshift_overrev(&mut self, target_gear: i32) -> (f64, bool) {
        let predicted_rpm = self.predict_downshift_rpm(self.wheel_speed, target_gear);
        if predicted_rpm > self.engine.rev_limiter_rpm {
            // Drivetrain-forced over-rev — rev limiter cannot help here
            self.overrev_events.push(OverrevCause::RapidDownshift);
            (predicted_rpm, true) // blocked
        } else if predicted_rpm > self.max_downshift_rpm {
            (predicted_rpm, true) // soft limit
        } else {
            (predicted_rpm, false) // safe
        }
    }

    pub fn set_wheel_radius(&mut self, radius: f64) {
        if radius.is_finite() && radius > 0.05 {
            self.wheel_radius = radius.clamp(0.05, 2.0);
        }
    }

    pub fn set_wheel_inertia(&mut self, inertia: f64) {
        if inertia.is_finite() && inertia > 0.0 {
            self.wheel_inertia = inertia.max(0.1);
        }
    }

    fn effective_wheel_inertia(&self, vehicle_mass: f64) -> f64 {
        if self.wheel_inertia.is_finite() && self.wheel_inertia > 0.0 {
            self.wheel_inertia
        } else {
            (vehicle_mass.max(1.0) * 0.01).max(0.1)
        }
    }

    /// Apply the equal-and-opposite reaction from the driven tire contact
    /// patch. `update` integrates the torque delivered to the wheel shaft;
    /// without this reaction the shaft can accelerate to its safety clamp
    /// while the chassis receives only the tire force, making traction control
    /// intervene forever and leaving wheel speed disconnected from road speed.
    pub fn apply_wheel_reaction_torque(&mut self, contact_torque: f64, dt: f64, vehicle_mass: f64) {
        if !contact_torque.is_finite() || !dt.is_finite() || dt <= 0.0 {
            return;
        }
        let wheel_inertia = self.effective_wheel_inertia(vehicle_mass);
        self.wheel_speed -= contact_torque / wheel_inertia * dt;
        self.wheel_speed = self.wheel_speed.clamp(-200.0, 200.0);
    }

    /// Resolve the driven shaft against grounded tire contact. Static contact
    /// constrains the shaft to rolling speed, while a saturated contact patch
    /// only applies its reaction torque and preserves accumulated wheelspin.
    /// Merely subtracting reaction torque in the static case leaves the shaft
    /// near zero while the chassis accelerates, making light automatic
    /// throttle look like locked tires on ice.
    pub fn apply_grounded_wheel_response(
        &mut self,
        requested_torque: f64,
        contact_torque: f64,
        rolling_speed: f64,
        dt: f64,
        vehicle_mass: f64,
    ) {
        if !requested_torque.is_finite()
            || !contact_torque.is_finite()
            || !rolling_speed.is_finite()
            || !dt.is_finite()
            || dt <= 0.0
        {
            return;
        }
        let torque_scale = requested_torque.abs().max(contact_torque.abs()).max(1.0);
        let has_static_grip = (requested_torque - contact_torque).abs() <= torque_scale * 1e-6;
        if has_static_grip {
            self.wheel_speed = rolling_speed.clamp(-200.0, 200.0);
        } else {
            self.apply_wheel_reaction_torque(contact_torque, dt, vehicle_mass);
        }
    }

    pub fn set_shift_duration_multiplier(&mut self, multiplier: f64) {
        if multiplier.is_finite() {
            self.shift_duration_multiplier = multiplier.clamp(1.0, 4.0);
        }
    }

    /// Compute clutch shock magnitude from RPM mismatch during shift completion.
    fn compute_clutch_shock(&mut self, dt: f64) {
        let target_rpm = self
            .transmission
            .engine_rpm_from_wheel_speed(self.wheel_speed);
        let mismatch = (self.engine.rpm - target_rpm).abs();
        self.clutch_shock
            .apply(mismatch, self.transmission.clutch_engagement, dt);
    }

    pub fn apply_automatic_shifting(&mut self) {
        if self.transmission.mode != TransmissionMode::Automatic {
            return;
        }
        if self.shift_phase != ShiftPhase::Idle {
            return;
        }
        if self.auto_shift.should_upshift(
            self.transmission.current_gear,
            self.engine.rpm,
            self.throttle_input,
            self.time,
        ) {
            self.request_shift_up();
        } else if self.auto_shift.should_downshift(
            self.transmission.current_gear,
            self.engine.rpm,
            self.time,
        ) {
            self.request_shift_down();
        }
    }

    pub fn set_gear(&mut self, gear: i32) {
        let max = self.transmission.gear_ratios.len() as i32;
        self.transmission.current_gear = gear.clamp(-1, max);
    }

    pub fn get_rpm(&self) -> f64 {
        self.engine.rpm
    }

    pub fn get_gear(&self) -> i32 {
        self.transmission.current_gear
    }

    pub fn get_vehicle_speed(&self) -> f64 {
        // Compatibility accessor only. This is tire circumferential speed,
        // not chassis speed; PhysicsWorld derives vehicle speed from nodes.
        self.wheel_speed * self.wheel_radius
    }

    pub fn get_clutch(&self) -> f64 {
        self.transmission.clutch_engagement
    }

    pub fn get_drive_torque(&self) -> f64 {
        self.last_drive_torque
    }

    pub fn set_torque_converter_state(&mut self, coupling: f64, torque_multiplier: f64) {
        self.converter_coupling = coupling.clamp(0.1, 1.0);
        self.converter_torque_multiplier = torque_multiplier.clamp(0.5, 1.2);
    }

    pub fn shift_up(&mut self) {
        self.request_shift_up();
    }

    pub fn shift_down(&mut self) {
        self.request_shift_down();
    }
}

impl Default for Drivetrain {
    fn default() -> Self {
        Self::new()
    }
}
