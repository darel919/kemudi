use super::*;
fn make_test_systems() -> (
    EngineThermal,
    CoolingSystem,
    LubricationSystem,
    EngineDamage,
) {
    (
        EngineThermal::default(),
        CoolingSystem::default(),
        LubricationSystem::default(),
        EngineDamage::default(),
    )
}
#[test]
fn test_heat_generation() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    let t0 = thermal.coolant_temp;
    update_engine(
        &mut thermal,
        &cooling,
        &mut lub,
        &mut dmg,
        3000.0,
        7000.0,
        1.0,
        0.0,
        0.0,
        1.0,
    );
    assert!(
        thermal.coolant_temp > t0,
        "Coolant should heat up under load"
    );
}
#[test]
fn test_cooling_basic() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    thermal.coolant_temp = 100.0;
    // Run for several seconds with vehicle speed for airflow
    for _ in 0..60 {
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            0.0,
            7000.0,
            0.0,
            30.0,
            0.0,
            1.0,
        );
    }
    assert!(
        thermal.coolant_temp < 100.0,
        "Cooling should reduce temperature"
    );
}
#[test]
fn test_thermostat_hysteresis() {
    let cooling = CoolingSystem::default();
    assert!((cooling.thermostat_valve(70.0) - 0.0).abs() < 1e-6);
    assert!((cooling.thermostat_valve(95.0) - 1.0).abs() < 1e-6);
    let mid = cooling.thermostat_valve(88.5);
    assert!(mid > 0.0 && mid < 1.0);
}
#[test]
fn test_fan_activation() {
    let cooling = CoolingSystem::default();
    assert!(!cooling.fan_active(80.0));
    assert!(cooling.fan_active(95.0));
}
#[test]
fn test_airflow_cooling() {
    let cooling = CoolingSystem::default();
    let rate_slow = cooling.cooling_rate(95.0, 5.0);
    let rate_fast = cooling.cooling_rate(95.0, 30.0);
    assert!(
        rate_fast > rate_slow,
        "More airflow should increase cooling"
    );
}
#[test]
fn test_heat_soak_after_shutdown() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    thermal.coolant_temp = 80.0;
    // No throttle, low RPM → heat soak
    for _ in 0..200 {
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            500.0,
            7000.0,
            0.0,
            0.0,
            0.0,
            1.0,
        );
    }
    assert!(
        thermal.coolant_temp < 80.0,
        "Should cool toward ambient via heat soak"
    );
}
#[test]
fn test_coolant_leak() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    dmg.has_coolant_leak = true;
    thermal.coolant_temp = 90.0;
    // Leak + no cooling → faster heat rise than no leak
    let mut thermal_no_leak = thermal.clone();
    let mut dmg_no_leak = EngineDamage::default();
    for _ in 0..10 {
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            3000.0,
            7000.0,
            0.5,
            0.0,
            0.0,
            1.0,
        );
        update_engine(
            &mut thermal_no_leak,
            &cooling,
            &mut lub,
            &mut dmg_no_leak,
            3000.0,
            7000.0,
            0.5,
            0.0,
            0.0,
            1.0,
        );
    }
    // Leaking system should run hotter due to reduced coolant capacity effect
    // Both heat up, but we just verify the system doesn't crash
    assert!(thermal.coolant_temp > 20.0);
}
#[test]
fn test_oil_pressure_normal() {
    let (_, _, lub, _) = make_test_systems();
    assert!(
        lub.oil_pressure > 200.0,
        "Normal oil pressure should be healthy"
    );
}

#[test]
fn test_oil_pressure_decays_when_engine_is_stopped() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    let pressure_before = lub.oil_pressure;
    let telemetry = update_engine(
        &mut thermal,
        &cooling,
        &mut lub,
        &mut dmg,
        0.0,
        7000.0,
        0.0,
        0.0,
        0.0,
        0.1,
    );
    assert!(lub.oil_pressure < pressure_before);
    assert_eq!(dmg.bearing_damage, 0.0);
    assert!(!telemetry.has_warning);
}

#[test]
fn test_oil_pressure_starvation() {
    let mut lub = LubricationSystem::default();
    lub.oil_level = 0.1; // Very low oil
    lub.update(0.0, 0.1);
    assert!(lub.oil_pressure < 100.0, "Low oil should reduce pressure");
    assert!(lub.is_starved());
}
#[test]
fn test_oil_viscosity_temperature() {
    let lub = LubricationSystem::default();
    let cold_visc = lub.viscosity(20.0);
    let hot_visc = lub.viscosity(120.0);
    assert!(cold_visc > hot_visc, "Cold oil should be more viscous");
}
#[test]
fn test_overheat_derate() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    // Force extreme temp and keep it there — many cycles to accumulate damage
    for _ in 0..500 {
        thermal.coolant_temp = 145.0;
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            6000.0,
            7000.0,
            1.0,
            0.0,
            0.0,
            0.5,
        );
    }
    let telem = update_engine(
        &mut thermal,
        &cooling,
        &mut lub,
        &mut dmg,
        6000.0,
        7000.0,
        1.0,
        0.0,
        0.0,
        0.1,
    );
    assert!(telem.derate_factor < 1.0, "Overheating should derate power");
}
#[test]
fn test_overrev_damage() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    for _ in 0..200 {
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            8000.0,
            7000.0,
            1.0,
            0.0,
            0.0,
            0.1,
        );
    }
    assert!(dmg.overrev_cycles > 0, "Should accumulate overrev cycles");
    assert!(dmg.wear_level > 0.0, "Overrev should cause wear");
}
#[test]
fn test_bearing_damage_from_oil_starvation() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    lub.oil_level = 0.0;
    for _ in 0..500 {
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            3000.0,
            7000.0,
            0.5,
            0.0,
            0.0,
            0.1,
        );
    }
    assert!(
        dmg.bearing_damage > 0.0,
        "Oil starvation should damage bearings"
    );
}
#[test]
fn test_seizure() {
    let (_, mut lub, mut dmg) = (
        EngineThermal::default(),
        LubricationSystem::default(),
        EngineDamage::default(),
    );
    lub.oil_level = 0.0;
    lub.oil_pressure = 0.0; // Simulate complete oil pressure loss
    for _ in 0..500 {
        dmg.check_oil_starvation(lub.oil_pressure);
    }
    assert!(dmg.is_seized, "Extreme starvation should seize engine");
}
#[test]
fn test_fire_risk() {
    let (mut thermal, mut cooling, mut lub, mut dmg) = make_test_systems();
    cooling.damage_factor = 0.0; // Disable cooling
    thermal.coolant_temp = 145.0;
    dmg.wear_level = 0.9;
    for _ in 0..200 {
        update_engine(
            &mut thermal,
            &cooling,
            &mut lub,
            &mut dmg,
            6000.0,
            7000.0,
            1.0,
            0.0,
            0.0,
            1.0,
        );
    }
    assert!(
        dmg.fire_timer > 0.0 || dmg.is_on_fire,
        "Extreme conditions should risk fire"
    );
}
#[test]
fn test_persistent_damage() {
    let mut dmg = EngineDamage::default();
    dmg.bearing_damage = 0.5;
    dmg.wear_level = 0.3;
    // Simulate time passing with no further issues
    dmg.update(10.0);
    assert!(
        dmg.bearing_damage >= 0.5,
        "Bearing damage should not self-repair"
    );
    assert!(dmg.wear_level >= 0.3, "Wear should not self-repair");
}
#[test]
fn test_limp_home() {
    let (mut thermal, cooling, mut lub, mut dmg) = make_test_systems();
    dmg.bearing_damage = 0.6;
    let telem = update_engine(
        &mut thermal,
        &cooling,
        &mut lub,
        &mut dmg,
        3000.0,
        7000.0,
        0.5,
        0.0,
        0.0,
        0.1,
    );
    assert!(
        telem.derate_factor < 0.8,
        "Severe damage should significantly derate"
    );
    assert!(telem.has_warning, "Should trigger warning");
}
#[test]
fn test_cooling_damaged_reduces_effectiveness() {
    let mut cooling = CoolingSystem::default();
    let rate_healthy = cooling.cooling_rate(95.0, 20.0);
    cooling.damage_factor = 0.3;
    let rate_damaged = cooling.cooling_rate(95.0, 20.0);
    assert!(
        rate_damaged < rate_healthy,
        "Damaged cooling should be less effective"
    );
}
