use super::{
    BlowupStage, CoolingSystem, DamageEvent, EngineDamage, EngineStressAccumulators, EngineThermal,
    LubricationSystem,
};
use serde::{Deserialize, Serialize};

/// Full telemetry output from the engine update cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineTelemetry {
    pub coolant_temp: f64,
    pub oil_temp: f64,
    pub oil_pressure: f64,
    pub transmission_temp: f64,
    pub derate_factor: f64,
    pub has_warning: bool,
    pub is_seized: bool,
    pub is_on_fire: bool,
    /// Current blow-up stage.
    pub blowup_stage: BlowupStage,
    /// Cumulative stress accumulator values for telemetry.
    pub thermal_stress: f64,
    pub oil_stress: f64,
    pub overrev_stress: f64,
    pub lugging_stress: f64,
    /// Recent causal damage events.
    pub damage_events: Vec<DamageEvent>,
}

/// Main update function for engine thermal, lubrication, and damage.
pub fn update_engine(
    thermal: &mut EngineThermal,
    cooling: &CoolingSystem,
    lubrication: &mut LubricationSystem,
    damage: &mut EngineDamage,
    rpm: f64,
    redline_rpm: f64,
    throttle: f64,
    vehicle_speed: f64,
    accel_g: f64,
    dt: f64,
) -> EngineTelemetry {
    // Heat generation
    let safe_redline = if redline_rpm.is_finite() && redline_rpm > 0.0 {
        redline_rpm
    } else {
        7000.0
    };
    let safe_throttle = if throttle.is_finite() {
        throttle.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let normalized_rpm = if rpm.is_finite() {
        (rpm / safe_redline).clamp(0.0, 1.5)
    } else {
        0.0
    };
    let load = safe_throttle * normalized_rpm;
    let combustion_heat = thermal.combustion_heat_rate * load;
    let friction_heat = thermal.friction_heat_rate * normalized_rpm;
    let total_heat = combustion_heat + friction_heat;
    let exhaust_fraction = thermal.exhaust_heat_fraction;
    let retained_heat = total_heat * (1.0 - exhaust_fraction);

    // Distribute heat: 60% to coolant, 40% to oil
    let coolant_heat = retained_heat * 0.6;
    let oil_heat = retained_heat * 0.4;

    let safe_dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
    let engine_running = rpm.is_finite() && rpm > 0.0;
    thermal.coolant_temp += coolant_heat
        / (thermal.coolant_capacity.max(0.1) * thermal.coolant_specific_heat.max(1.0))
        * safe_dt;
    thermal.oil_temp +=
        oil_heat / (thermal.oil_capacity.max(0.1) * thermal.oil_specific_heat.max(1.0)) * safe_dt;

    // Cooling
    let cool_rate = cooling.cooling_rate(thermal.coolant_temp, vehicle_speed);
    let ambient_delta = (thermal.coolant_temp - thermal.ambient_temp).max(0.0);
    thermal.coolant_temp -= cool_rate
        / (thermal.coolant_capacity.max(0.1) * thermal.coolant_specific_heat.max(1.0))
        * ambient_delta
        * safe_dt;

    // Heat soak toward ambient when engine off
    if throttle < 0.01 && rpm < 1000.0 {
        let soak_rate = 0.01;
        thermal.coolant_temp += (thermal.ambient_temp - thermal.coolant_temp) * soak_rate * safe_dt;
        thermal.oil_temp += (thermal.ambient_temp - thermal.oil_temp) * soak_rate * safe_dt;
    }

    // Oil temp approaches coolant temp (heat exchange)
    thermal.oil_temp += (thermal.coolant_temp - thermal.oil_temp) * 0.001 * safe_dt;

    // Transmission temp follows oil temp loosely
    thermal.transmission_temp += (thermal.oil_temp - thermal.transmission_temp) * 0.0005 * safe_dt;

    // Clamp temps
    thermal.coolant_temp = thermal.coolant_temp.max(thermal.ambient_temp - 5.0);
    thermal.oil_temp = thermal.oil_temp.max(thermal.ambient_temp - 5.0);

    // Lubrication update
    lubrication.update_with_engine_rpm(accel_g, dt.max(0.0), rpm);

    // Damage checks
    damage.check_overheat(thermal.coolant_temp);
    damage.check_overrev(rpm, redline_rpm);
    if rpm.is_finite() && rpm > 0.0 {
        damage.check_oil_starvation(lubrication.oil_pressure);
    }
    damage.update(dt);

    // Derate factor
    let derate_factor = if damage.is_seized {
        0.0
    } else {
        let bearing_derate = 1.0 - damage.bearing_damage * 0.5;
        let wear_derate = 1.0 - damage.wear_level * 0.3;
        let gasket_derate = if damage.head_gasket_failed { 0.6 } else { 1.0 };
        bearing_derate.max(0.0) * wear_derate.max(0.0) * gasket_derate
    };

    // Warning state
    let has_warning = thermal.coolant_temp > 105.0
        || (engine_running && lubrication.oil_pressure < 80.0)
        || damage.bearing_damage > 0.1
        || damage.is_on_fire;

    EngineTelemetry {
        coolant_temp: thermal.coolant_temp,
        oil_temp: thermal.oil_temp,
        oil_pressure: lubrication.oil_pressure,
        transmission_temp: thermal.transmission_temp,
        derate_factor,
        has_warning,
        is_seized: damage.is_seized,
        is_on_fire: damage.is_on_fire,
        blowup_stage: BlowupStage::Ok,
        thermal_stress: 0.0,
        oil_stress: 0.0,
        overrev_stress: 0.0,
        lugging_stress: 0.0,
        damage_events: Vec::new(),
    }
}

/// Extended engine update with stress accumulators, lugging detection, and staged blow-up.
pub fn update_engine_full(
    thermal: &mut EngineThermal,
    cooling: &CoolingSystem,
    lubrication: &mut LubricationSystem,
    damage: &mut EngineDamage,
    stress: &mut EngineStressAccumulators,
    rpm: f64,
    redline_rpm: f64,
    throttle: f64,
    vehicle_speed: f64,
    accel_g: f64,
    gear: i32,
    time: f64,
    dt: f64,
) -> EngineTelemetry {
    update_engine_full_with_cause(
        thermal,
        cooling,
        lubrication,
        damage,
        stress,
        rpm,
        redline_rpm,
        throttle,
        vehicle_speed,
        accel_g,
        gear,
        time,
        dt,
        false,
    )
}

/// Full engine update with a causal drivetrain over-rev signal.
///
/// The compatibility wrapper above keeps existing callers deterministic,
/// while the authoritative vehicle world can distinguish limiter revving from
/// a forced over-rev caused by a driveline event.
pub fn update_engine_full_with_cause(
    thermal: &mut EngineThermal,
    cooling: &CoolingSystem,
    lubrication: &mut LubricationSystem,
    damage: &mut EngineDamage,
    stress: &mut EngineStressAccumulators,
    rpm: f64,
    redline_rpm: f64,
    throttle: f64,
    vehicle_speed: f64,
    accel_g: f64,
    gear: i32,
    time: f64,
    dt: f64,
    drivetrain_forced_overrev: bool,
) -> EngineTelemetry {
    // Run the base update
    let base = update_engine(
        thermal,
        cooling,
        lubrication,
        damage,
        rpm,
        redline_rpm,
        throttle,
        vehicle_speed,
        accel_g,
        dt,
    );

    // Accumulate stress
    stress.accumulate_thermal(thermal.coolant_temp, 110.0, dt);
    if rpm.is_finite() && rpm > 0.0 {
        stress.accumulate_oil(lubrication.oil_pressure, lubrication.nominal_pressure, dt);
    } else {
        stress.oil_stress = (stress.oil_stress - stress.oil_cooldown_rate * dt.max(0.0)).max(0.0);
    }
    stress.accumulate_overrev(rpm, redline_rpm, drivetrain_forced_overrev, dt);
    stress.accumulate_lugging(rpm, throttle, gear, 1500.0, dt);

    // Check lugging damage
    damage.check_lugging(rpm, throttle, gear, time);

    // Determine blow-up stage from stress
    let blowup = stress.blowup_stage();
    let stress_derate = stress.derate_factor();

    // Combined derate: base damage × stress derate
    let derate_factor = base.derate_factor * stress_derate;

    // Warning includes stress
    let has_warning = base.has_warning || blowup as u8 >= BlowupStage::WarningLamp as u8;

    EngineTelemetry {
        coolant_temp: base.coolant_temp,
        oil_temp: base.oil_temp,
        oil_pressure: base.oil_pressure,
        transmission_temp: base.transmission_temp,
        derate_factor,
        has_warning,
        is_seized: base.is_seized || blowup as u8 >= BlowupStage::SeizureOrFire as u8,
        is_on_fire: base.is_on_fire,
        blowup_stage: blowup,
        thermal_stress: stress.thermal_stress,
        oil_stress: stress.oil_stress,
        overrev_stress: stress.overrev_stress,
        lugging_stress: stress.lugging_stress,
        damage_events: damage.damage_history.clone(),
    }
}
