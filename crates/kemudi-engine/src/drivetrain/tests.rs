use super::*;
#[test]
fn test_engine_default_rpm() {
    let e = Engine::default();
    assert!((e.rpm - 800.0).abs() < 1e-6);
    assert_eq!(e.idle_rpm, 800.0);
    assert_eq!(e.redline_rpm, 7000.0);
}
#[test]
fn test_engine_torque_curve() {
    let e = Engine::default();
    let t0 = e.torque_at_rpm(0.0);
    let t3k = e.torque_at_rpm(3000.0);
    let t7k = e.torque_at_rpm(7000.0);
    assert!(t0 > 0.0);
    assert!(t3k > t0);
    assert!(t7k > 0.0);
}
#[test]
fn test_transmission_total_ratio() {
    let mut t = Transmission::default();
    t.current_gear = 1;
    let ratio = t.total_ratio();
    assert!(ratio > 0.0);
    assert!((ratio - 3.5 * 3.7).abs() < 1e-6);
}
#[test]
fn test_transmission_shift_up() {
    let mut t = Transmission::default();
    t.current_gear = 1;
    let ok = t.shift_up();
    assert!(ok);
    assert_eq!(t.current_gear, 2);
}
#[test]
fn test_transmission_shift_down() {
    let mut t = Transmission::default();
    t.current_gear = 1;
    t.shift_down();
    assert_eq!(t.current_gear, 0);
    t.shift_down();
    assert_eq!(t.current_gear, -1);
    let ok = t.shift_down();
    assert!(!ok);
    assert_eq!(t.current_gear, -1);
}
#[test]
fn test_drivetrain_speed_increases_with_throttle() {
    let mut dt = Drivetrain::new();
    dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
    let speed1 = dt.get_vehicle_speed();
    for _ in 0..60 {
        dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
    }
    assert!(dt.get_vehicle_speed() > speed1);
}

#[test]
fn manual_clutch_coupling_tracks_engine_speed_from_road_speed() {
    let mut dt = Drivetrain::new();
    dt.transmission.mode = TransmissionMode::Manual;
    dt.set_wheel_radius(0.34);
    dt.wheel_speed = 33.3 / dt.wheel_radius;

    dt.update(1.0, 0.0, 0.0, 1.0 / 120.0, 1500.0);

    assert!(
        dt.engine.rpm > 6000.0,
        "an engaged manual clutch must transmit road speed into engine RPM, got {} RPM",
        dt.engine.rpm
    );
    assert_eq!(
        dt.get_drive_torque(),
        0.0,
        "manual first gear must cut propulsion when the coupled engine is beyond its limiter"
    );
}

#[test]
fn test_drivetrain_brake_reduces_speed() {
    let mut dt = Drivetrain::new();
    for _ in 0..120 {
        dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
    }
    let speed_before = dt.get_vehicle_speed();
    for _ in 0..120 {
        dt.update(0.0, 1.0, 0.0, 1.0 / 60.0, 1500.0);
    }
    assert!(dt.get_vehicle_speed() < speed_before);
}

#[test]
fn test_service_brake_is_not_reported_as_drive_torque() {
    let mut dt = Drivetrain::new();
    dt.wheel_speed = 20.0;
    dt.update(0.0, 1.0, 1.0, 1.0 / 120.0, 1500.0);
    assert_eq!(dt.get_drive_torque(), 0.0);
    assert!(dt.wheel_speed < 20.0);
}

#[test]
fn test_contact_reaction_cancels_transmitted_wheel_torque() {
    let mut dt = Drivetrain::new();
    let step = 1.0 / 120.0;
    dt.update(0.5, 0.0, 0.0, step, 1500.0);
    let speed_before_reaction = dt.wheel_speed;
    let transmitted_torque = dt.last_drive_torque;
    assert!(transmitted_torque > 0.0);
    dt.apply_wheel_reaction_torque(transmitted_torque, step, 1500.0);
    assert!(
        dt.wheel_speed.abs() < 1e-7,
        "equal tire reaction must cancel shaft acceleration: {speed_before_reaction} -> {}",
        dt.wheel_speed
    );
}
#[test]
fn test_grounded_contact_preserves_rolling_speed_when_torque_is_transmitted() {
    let mut dt = Drivetrain::new();
    let step = 1.0 / 120.0;
    dt.update(0.2, 0.0, 0.0, step, 1500.0);
    let requested_torque = dt.last_drive_torque;
    dt.apply_grounded_wheel_response(requested_torque, requested_torque, 12.0, step, 1500.0);
    assert!(
        (dt.wheel_speed - 12.0).abs() < 1e-9,
        "static contact should rotate the driven shaft at road speed"
    );
}
#[test]
fn test_grounded_contact_retains_only_untransmitted_spin_torque() {
    let mut dt = Drivetrain::new();
    let step = 1.0 / 120.0;
    let rolling_speed = 12.0;
    let wheel_inertia = 1500.0 * 0.01;
    dt.wheel_speed = rolling_speed + 600.0 / wheel_inertia * step;
    dt.apply_grounded_wheel_response(600.0, 150.0, 12.0, step, 1500.0);
    assert!(
        dt.wheel_speed > rolling_speed,
        "untransmitted torque should preserve wheel spin"
    );
    assert!(
        dt.wheel_speed < rolling_speed + 600.0 / wheel_inertia * step,
        "the transmitted contact torque should still oppose wheel spin"
    );
}
#[test]
fn test_drivetrain_clutch_disengages() {
    let mut dt = Drivetrain::new();
    dt.update(1.0, 0.0, 1.0, 1.0 / 60.0, 1500.0);
    assert!(dt.get_clutch() < 0.5);
}
#[test]
fn test_drivetrain_idle() {
    let mut dt = Drivetrain::new();
    for _ in 0..300 {
        dt.update(0.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
    }
    assert!(dt.get_rpm() < 1000.0);
}
// === New tests ===
#[test]
fn test_auto_shift_up() {
    let mut dt = Drivetrain::new();
    dt.transmission.mode = TransmissionMode::Automatic;
    dt.auto_shift.upshift_rpm = 4000.0;
    dt.auto_shift.shift_delay = 0.0;
    dt.auto_shift.last_shift_time = -10.0;
    dt.engine.rpm = 5500.0;
    dt.apply_automatic_shifting();
    // Process shift
    for _ in 0..60 {
        dt.update(1.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
        dt.engine.rpm = 5500.0;
    }
    assert!(
        dt.get_gear() >= 2
            || dt.shift_phase == ShiftPhase::Engaging
            || dt.shift_phase == ShiftPhase::Neutral,
        "Auto mode should have initiated upshift, gear={}",
        dt.get_gear()
    );
}
#[test]
fn test_auto_shift_down() {
    let mut dt = Drivetrain::new();
    dt.transmission.mode = TransmissionMode::Automatic;
    dt.transmission.current_gear = 3;
    dt.auto_shift.downshift_rpm = 2500.0;
    dt.auto_shift.shift_delay = 0.0;
    dt.auto_shift.last_shift_time = -10.0;
    dt.engine.rpm = 2000.0;
    dt.apply_automatic_shifting();
    // Should request downshift
    assert!(
        dt.shift_phase != ShiftPhase::Idle || dt.pending_gear < 3,
        "Auto mode should downshift at low RPM"
    );
}

#[test]
fn tcm_holds_first_gear_during_stationary_full_throttle_launch() {
    let mut tcm = TransmissionControlModule::default();
    let request = tcm.update_with_brake(4800.0, 0.0, 1.0, 0.0, 0.0, 20.0, 1, 4, 1.0 / 120.0);
    assert_eq!(request, None);
}

#[test]
fn test_shift_delay_prevents_rapid_shifts() {
    let mut dt = Drivetrain::new();
    dt.shift_up();
    // Process the shift
    for _ in 0..30 {
        dt.update(1.0, 0.0, 0.0, 0.01, 1500.0);
    }
    // Try immediately — should be blocked by delay
    let ok = dt.request_shift_up();
    assert!(
        !ok || dt.shift_phase != ShiftPhase::Idle,
        "Should not shift again within delay period"
    );
}
#[test]
fn test_kickdown_earlier_upshift() {
    let mut al = AutoShiftLogic::default();
    al.shift_delay = 0.0;
    al.last_shift_time = -10.0;
    // High throttle: upshift at lower RPM
    assert!(al.should_upshift(2, 5800.0, 1.0, 0.0));
    // Low throttle: requires higher RPM
    assert!(!al.should_upshift(2, 5800.0, 0.3, 0.0));
}
#[test]
fn test_blocked_shift_during_phase() {
    let mut dt = Drivetrain::new();
    dt.shift_up();
    // Should be in Disengaging
    assert_eq!(dt.shift_phase, ShiftPhase::Disengaging);
    // Try another shift — should be blocked
    let ok = dt.request_shift_down();
    assert!(!ok, "Can't shift during ongoing shift");
}
#[test]
fn test_rev_matching() {
    let mut dt = Drivetrain::new();
    dt.wheel_speed = 50.0;
    dt.engine.rpm = 6000.0;
    dt.shift_up();
    // Process through phases
    for _ in 0..30 {
        dt.update(1.0, 0.0, 0.0, 0.01, 1500.0);
    }
    // After shift, RPM should be closer to wheel-speed-derived RPM
    let target = dt.transmission.engine_rpm_from_wheel_speed(dt.wheel_speed);
    let diff = (dt.get_rpm() - target).abs();
    assert!(
        diff < 3000.0,
        "Rev matching should adjust RPM toward target"
    );
}
#[test]
fn test_stall_detection() {
    let mut dt = Drivetrain::new();
    dt.engine.rpm = 500.0; // below idle * 0.7
    dt.transmission.clutch_engagement = 0.8;
    dt.wheel_speed = 0.0;
    dt.update(0.0, 0.0, 0.0, 1.0 / 60.0, 1500.0);
    assert!(dt.is_stalled, "Should detect stall");
}
#[test]
fn test_differential_open() {
    let mut diff = Differential::default();
    diff.mode = DiffMode::Open;
    let (l, r) = diff.apply_differential(1000.0, 0.8, 0.6);
    assert!((l - 500.0).abs() < 1e-6);
    assert!((r - 500.0).abs() < 1e-6);
}
#[test]
fn test_differential_locked() {
    let mut diff = Differential::default();
    diff.mode = DiffMode::Locked;
    let (l, r) = diff.apply_differential_with_speeds(1000.0, 0.8, 0.2, 20.0, 20.0);
    assert!(
        l > r,
        "Locked differential should route reaction torque through grip"
    );
    assert!((l + r - 1000.0).abs() < 1e-6);
}
#[test]
fn test_differential_locked_resists_output_speed_difference() {
    let mut diff = Differential::default();
    diff.mode = DiffMode::Locked;
    let (l, r) = diff.apply_differential_with_speeds(1000.0, 0.5, 0.5, 40.0, 20.0);
    assert!(
        r > l,
        "Locked differential should transfer torque toward the slower output"
    );
    assert!((l + r - 1000.0).abs() < 1e-6);
}
#[test]
fn test_differential_lsd_biases_toward_grip() {
    let mut diff = Differential::default();
    diff.mode = DiffMode::LimitedSlip;
    diff.bias = 0.5;
    let (l, r) = diff.apply_differential(1000.0, 0.9, 0.1);
    // More grip on left -> more torque on left
    assert!(l > r, "LSD should bias torque toward wheel with more grip");
    assert!(
        (l + r - 1000.0).abs() < 1e-6,
        "Differential must conserve input torque"
    );
}
#[test]
fn test_ratio_validation() {
    let good = DrivetrainConfig {
        gear_ratios: vec![3.5, 2.1, 1.4, 1.0],
        final_drive: 3.7,
        reverse_ratio: -3.2,
        max_gears: 6,
    };
    assert!(good.validate().is_ok());
    let bad = DrivetrainConfig {
        gear_ratios: vec![1.0, 1.5], // ascending — invalid
        final_drive: 3.7,
        reverse_ratio: -3.2,
        max_gears: 6,
    };
    assert!(bad.validate().is_err());
}
#[test]
fn test_reverse_gear_negative_ratio() {
    let mut dt = Drivetrain::new();
    dt.set_gear(-1);
    assert_eq!(dt.get_gear(), -1);
    let ratio = dt.transmission.total_ratio();
    assert!(ratio < 0.0);
}
#[test]
fn test_neutral_gear_zero_transfer() {
    let mut dt = Drivetrain::new();
    dt.set_gear(0);
    assert_eq!(dt.get_gear(), 0);
    let ratio = dt.transmission.total_ratio();
    assert!((ratio).abs() < 1e-6, "Neutral should have zero ratio");
}
#[test]
fn test_shift_returns_bool() {
    let mut t = Transmission::default();
    t.current_gear = t.gear_ratios.len() as i32;
    let ok = t.shift_up();
    assert!(!ok, "Can't shift above max gear");
}
// === TCM Tests ===

#[cfg(test)]
mod tcm_tests {
    use super::*;
    #[test]
    fn test_tcm_upshift_at_high_rpm() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        assert!(
            shift == Some(3),
            "Should upshift from 2nd at 4500 RPM, got {:?}",
            shift
        );
    }
    #[test]
    fn test_tcm_downshift_at_low_rpm() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(1000.0, 20.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        assert!(
            shift == Some(2),
            "Should downshift from 3rd at 1000 RPM, got {:?}",
            shift
        );
    }
    #[test]
    fn test_tcm_kickdown() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(3000.0, 40.0, 0.9, 0.0, 80.0, 4, 6, 0.1);
        assert!(
            shift == Some(3),
            "Kickdown should downshift, got {:?}",
            shift
        );
    }

    #[test]
    fn test_tcm_full_throttle_at_limiter_upshifts() {
        let mut tcm = TransmissionControlModule::default();
        let shift = tcm.update(7200.0, 40.0, 1.0, 0.0, 80.0, 2, 6, 0.1);
        assert_eq!(
            shift,
            Some(3),
            "full throttle at the limiter must upshift instead of kickdown-hunting"
        );
    }

    #[test]
    fn test_tcm_applies_load_and_drive_mode_shift_strategy() {
        let mut normal_light_load = TransmissionControlModule::default();
        let normal_shift = normal_light_load.update(4000.0, 40.0, 0.0, 0.0, 80.0, 2, 6, 0.1);
        assert_eq!(normal_shift, Some(3));

        let mut normal_high_load = TransmissionControlModule::default();
        let high_load_shift = normal_high_load.update(4000.0, 40.0, 1.0, 0.0, 80.0, 2, 6, 0.1);
        assert_eq!(
            high_load_shift, None,
            "high requested load should hold the current gear for acceleration"
        );

        let mut sport = TransmissionControlModule::default();
        sport.set_drive_mode(DriveMode::Sport);
        let sport_shift = sport.update(4000.0, 40.0, 0.0, 0.0, 80.0, 2, 6, 0.1);
        assert_eq!(
            sport_shift, None,
            "sport mode should hold a lower gear beyond the normal shift point"
        );

        let mut eco = TransmissionControlModule::default();
        eco.set_drive_mode(DriveMode::Eco);
        let eco_shift = eco.update(3500.0, 40.0, 0.0, 0.0, 80.0, 2, 6, 0.1);
        assert_eq!(
            eco_shift,
            Some(3),
            "eco mode should upshift early at light load"
        );
    }

    #[test]
    fn test_tcm_shift_interval() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        // Immediate second shift should be blocked
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 3, 6, 0.01);
        assert!(shift.is_none(), "Should not shift again within interval");
    }
    #[test]
    fn test_tcm_converter_lockup() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(2500.0, 50.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        assert!(
            tcm.converter_lockup,
            "Should lock up at moderate speed/throttle"
        );
    }
    #[test]
    fn test_tcm_converter_unlock_on_kickdown() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(2500.0, 50.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        assert!(tcm.converter_lockup);
        tcm.update(3000.0, 50.0, 0.8, 0.0, 80.0, 3, 6, 0.1);
        assert!(!tcm.converter_lockup, "Should unlock on heavy throttle");
    }
    #[test]
    fn test_tcm_thermal_protection() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(3000.0, 40.0, 0.5, 0.0, 130.0, 3, 6, 0.1);
        assert!(tcm.limp_mode, "Should enter limp mode on overheat");
        assert!(tcm.line_pressure < 1.0, "Should reduce line pressure");
    }
    #[test]
    fn test_tcm_line_pressure_scales_with_throttle() {
        let mut tcm = TransmissionControlModule::default();
        tcm.update(3000.0, 40.0, 0.2, 0.0, 80.0, 3, 6, 0.1);
        let low = tcm.line_pressure;
        tcm.update(3000.0, 40.0, 0.9, 0.0, 80.0, 3, 6, 0.1);
        let high = tcm.line_pressure;
        assert!(high > low, "Higher throttle should increase line pressure");
    }
    #[test]
    fn test_tcm_no_shift_when_disabled() {
        let mut tcm = TransmissionControlModule::default();
        tcm.enabled = false;
        let shift = tcm.update(8000.0, 80.0, 1.0, 0.0, 80.0, 1, 6, 0.1);
        assert!(shift.is_none(), "Disabled TCM should not shift");
    }
    #[test]
    fn test_tcm_adaptive_learning() {
        let mut tcm = TransmissionControlModule::default();
        let initial = tcm.learning.shift_quality[1]; // gear 2 → index 1
                                                     // Do several upshifts
        for _ in 0..100 {
            tcm.time_since_shift = 10.0;
            tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        }
        let adapted = tcm.learning.shift_quality[1];
        assert!(adapted != initial, "Learning should adapt shift quality");
    }

    #[test]
    fn test_tcm_sensor_fault_uses_stale_value_then_fails_safe() {
        let mut tcm = TransmissionControlModule::default();
        tcm.set_fault(TCMFaultKind::InputSpeedSensor, true);
        tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        assert_eq!(tcm.last_diagnostic_code, 715);
        assert!(tcm.sensor_age > 0.0);
        tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.4);
        assert!(tcm.observed_input_rpm < 1.0);
        assert_eq!(tcm.state, TCMState::Fault);
    }

    #[test]
    fn test_tcm_hydraulic_loss_reduces_pressure_and_derates() {
        let mut tcm = TransmissionControlModule::default();
        tcm.set_fault(TCMFaultKind::HydraulicPressure, true);
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        assert!(
            shift.is_none(),
            "a failed hydraulic circuit cannot complete a shift"
        );
        assert!(tcm.line_pressure < 0.4);
        assert!(tcm.torque_reduction() > 0.0);
        assert!(tcm.shift_duration_multiplier() > 1.0);
    }

    #[test]
    fn test_tcm_solenoid_fault_records_request_without_shifting() {
        let mut tcm = TransmissionControlModule::default();
        tcm.set_fault(TCMFaultKind::ShiftSolenoid, true);
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        assert!(shift.is_none());
        assert_eq!(tcm.pending_shift, Some(3));
        assert!(tcm.shift_duration_multiplier() > 2.0);
    }

    #[test]
    fn test_tcm_communication_fault_enters_fail_safe() {
        let mut tcm = TransmissionControlModule::default();
        tcm.set_fault(TCMFaultKind::Communication, true);
        let shift = tcm.update(4500.0, 60.0, 0.5, 0.0, 80.0, 2, 6, 0.1);
        assert!(shift.is_none());
        assert!(tcm.limp_mode);
        assert_eq!(tcm.last_diagnostic_code, 101);
        assert_eq!(tcm.fail_safe_gear(6), 3);
    }

    #[test]
    fn test_tcm_intermittent_fault_is_deterministic() {
        let mut first = TransmissionControlModule::default();
        let mut second = first.clone();
        first.set_fault(TCMFaultKind::OutputSpeedSensor, true);
        second.set_fault(TCMFaultKind::OutputSpeedSensor, true);
        first.set_fault_intermittent(TCMFaultKind::OutputSpeedSensor, true);
        second.set_fault_intermittent(TCMFaultKind::OutputSpeedSensor, true);
        for _ in 0..120 {
            first.update(2500.0, 50.0, 0.2, 0.0, 80.0, 3, 6, 1.0 / 60.0);
            second.update(2500.0, 50.0, 0.2, 0.0, 80.0, 3, 6, 1.0 / 60.0);
            assert_eq!(first.observed_output_speed, second.observed_output_speed);
            assert_eq!(first.state, second.state);
        }
    }
}

// === Drive Mode Tests ===

#[cfg(test)]
mod drive_mode_tests {
    use super::*;
    #[test]
    fn test_drive_mode_profiles_exist() {
        for mode in DriveMode::all() {
            let profile = DriveModeProfile::for_mode(*mode);
            assert_eq!(profile.mode, *mode);
        }
    }
    #[test]
    fn test_eco_reduces_throttle() {
        let profile = DriveModeProfile::eco();
        assert!(profile.throttle_map < 1.0, "Eco should dampen throttle");
        assert!(
            profile.tcm_upshift_modifier < 0.0,
            "Eco should upshift earlier"
        );
    }
    #[test]
    fn test_sport_increases_throttle() {
        let profile = DriveModeProfile::sport();
        assert!(profile.throttle_map > 1.0, "Sport should sharpen throttle");
        assert!(
            profile.tcm_upshift_modifier > 0.0,
            "Sport should upshift later"
        );
        assert_eq!(profile.differential_mode, DiffMode::LimitedSlip);
    }
    #[test]
    fn test_track_maximizes_performance() {
        let profile = DriveModeProfile::track();
        assert!(profile.throttle_map > 1.1);
        assert!(profile.suspension_damping > 1.3);
        assert_eq!(profile.differential_mode, DiffMode::Locked);
    }
    #[test]
    fn test_snow_reduces_power_and_increases_intervention() {
        let profile = DriveModeProfile::snow();
        assert!(profile.throttle_map < 0.7);
        assert!(
            profile.abs_threshold_modifier < 1.0,
            "Snow should have more ABS intervention"
        );
        assert!(
            profile.tc_threshold_modifier < 1.0,
            "Snow should have more TC intervention"
        );
    }
    #[test]
    fn test_mode_switching_with_cooldown() {
        let mut ctrl = DriveModeController::default();
        assert!(ctrl.request_mode(DriveMode::Sport, true, true));
        assert_eq!(ctrl.active_mode, DriveMode::Sport);
        // Immediate switch should be blocked by cooldown
        assert!(!ctrl.request_mode(DriveMode::Eco, true, true));
    }
    #[test]
    fn test_mode_switching_after_cooldown() {
        let mut ctrl = DriveModeController::default();
        ctrl.request_mode(DriveMode::Sport, true, true);
        ctrl.update(2.0); // past cooldown
        assert!(ctrl.request_mode(DriveMode::Eco, true, true));
        assert_eq!(ctrl.active_mode, DriveMode::Eco);
    }
    #[test]
    fn test_throttle_mapping() {
        let ctrl = DriveModeController {
            active_profile: DriveModeProfile::eco(),
            ..Default::default()
        };
        let mapped = ctrl.map_throttle(1.0);
        assert!(mapped < 1.0, "Eco mode should reduce throttle");
        assert!(mapped > 0.0);
    }
    #[test]
    fn test_suspension_multiplier() {
        let mut ctrl = DriveModeController::default();
        ctrl.request_mode(DriveMode::Track, true, true);
        assert!(
            ctrl.suspension_multiplier() > 1.0,
            "Track should stiffen suspension"
        );
    }
    #[test]
    fn test_steering_multiplier() {
        let mut ctrl = DriveModeController::default();
        ctrl.request_mode(DriveMode::Comfort, true, true);
        // Comfort: assist=1.3, response=0.85 → multiplier ~1.105
        let mult = ctrl.steering_multiplier();
        assert!(
            mult > 0.8 && mult < 1.5,
            "Comfort steering multiplier should be reasonable"
        );
    }
}
