use serde::{Deserialize, Serialize};

/// Tire compound type with physics parameters.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum TireCompound {
    Sport,
    Street,
    Offroad,
    Mud,
    Snow,
}

impl TireCompound {
    pub fn base_grip(&self) -> f64 {
        match self {
            Self::Sport => 0.95,
            Self::Street => 0.85,
            Self::Offroad => 0.7,
            Self::Mud => 0.5,
            Self::Snow => 0.4,
        }
    }
    pub fn temp_sensitivity(&self) -> f64 {
        match self {
            Self::Sport => 1.2,
            Self::Street => 1.0,
            Self::Offroad => 0.8,
            Self::Mud => 0.6,
            Self::Snow => 0.7,
        }
    }
    pub fn wear_rate(&self) -> f64 {
        match self {
            Self::Sport => 1.5,
            Self::Street => 1.0,
            Self::Offroad => 0.8,
            Self::Mud => 0.7,
            Self::Snow => 0.6,
        }
    }
    pub fn optimal_temp_min(&self) -> f64 {
        match self {
            Self::Sport => 80.0,
            Self::Street => 60.0,
            Self::Offroad => 50.0,
            Self::Mud => 40.0,
            Self::Snow => 30.0,
        }
    }
    pub fn optimal_temp_max(&self) -> f64 {
        match self {
            Self::Sport => 120.0,
            Self::Street => 100.0,
            Self::Offroad => 90.0,
            Self::Mud => 80.0,
            Self::Snow => 70.0,
        }
    }
}

/// Per-tire state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TireState {
    pub compound: TireCompound,
    pub temperature: f64,
    pub pressure: f64,
    pub nominal_pressure: f64,
    pub wear: f64,
    pub grip_factor: f64,
    pub is_flat: bool,
    pub has_blister: bool,
    pub has_puncture: bool,
    pub cord_exposed: bool,
    pub blowout_risk: f64,
    pub center_wear: f64,
    pub shoulder_wear: f64,
}

impl Default for TireState {
    fn default() -> Self {
        Self {
            compound: TireCompound::Street,
            temperature: 20.0,
            pressure: 220.0,
            nominal_pressure: 220.0,
            wear: 0.0,
            grip_factor: 0.85,
            is_flat: false,
            has_blister: false,
            has_puncture: false,
            cord_exposed: false,
            blowout_risk: 0.0,
            center_wear: 0.0,
            shoulder_wear: 0.0,
        }
    }
}

/// Thermal parameters for heat generation/dissipation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TireThermalParams {
    pub heat_generation_rate: f64,
    pub heat_dissipation_rate: f64,
    pub thermal_mass: f64,
}

impl Default for TireThermalParams {
    fn default() -> Self {
        Self {
            heat_generation_rate: 5.0,
            heat_dissipation_rate: 50.0,
            thermal_mass: 8.0,
        }
    }
}

/// Failure type.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum TireFailure {
    None,
    FlatSpot,
    Puncture,
    SidewallDamage,
    Blowout,
}

/// Telemetry snapshot for one tire.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TireTelemetry {
    pub temperature: f64,
    pub pressure: f64,
    pub wear: f64,
    pub grip_factor: f64,
    pub is_flat: bool,
    pub failure: TireFailure,
    pub center_wear: f64,
    pub shoulder_wear: f64,
}

/// Compact, allocation-free tire force output from the Magic Formula model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacejkaForceResult {
    /// Force along the wheel rolling direction. Positive force follows
    /// positive wheel slip (the wheel is driving the vehicle forward).
    pub longitudinal: f64,
    /// Force along the lateral contact axis. Positive slip angle produces a
    /// negative restoring force by convention.
    pub lateral: f64,
    /// Available pure longitudinal peak force before combined-slip weighting.
    pub longitudinal_peak: f64,
    /// Available pure lateral peak force before combined-slip weighting.
    pub lateral_peak: f64,
    /// Magnitude of the final combined force vector.
    pub combined_magnitude: f64,
    /// Combined force budget for the contact patch.
    pub combined_limit: f64,
}

/// Separate Magic Formula parameters for the two tire force directions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacejkaParameters {
    pub longitudinal_b: f64,
    pub longitudinal_c: f64,
    pub longitudinal_d: f64,
    pub longitudinal_e: f64,
    pub lateral_b: f64,
    pub lateral_c: f64,
    pub lateral_d: f64,
    pub lateral_e: f64,
}

impl Default for PacejkaParameters {
    fn default() -> Self {
        Self {
            // The lateral shape uses a higher C so the force peaks around a
            // realistic slip angle and rolls off under gross cornering slip.
            longitudinal_b: 10.0,
            longitudinal_c: 1.65,
            longitudinal_d: 1.0,
            longitudinal_e: 0.97,
            lateral_b: 10.0,
            lateral_c: 1.9,
            lateral_d: 1.0,
            lateral_e: 0.97,
        }
    }
}

const REFERENCE_TIRE_LOAD: f64 = 4_000.0;

fn safe_finite(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

fn magic_formula(slip: f64, b: f64, c: f64, d: f64, e: f64) -> f64 {
    let safe_slip = safe_finite(slip).clamp(-4.0, 4.0);
    let safe_b = safe_finite(b).max(0.0);
    let safe_c = safe_finite(c).max(0.0);
    let safe_d = safe_finite(d).max(0.0);
    let safe_e = safe_finite(e).clamp(-1.0, 1.0);
    let bx = safe_b * safe_slip;
    let value = safe_d * (safe_c * (bx - safe_e * (bx - bx.atan())).atan()).sin();
    safe_finite(value)
}

/// Calculate load-sensitive longitudinal and lateral tire forces.
///
/// The model uses separate longitudinal/lateral B, C, D, and E parameters,
/// scales peak force with normal load and available surface/tire grip, then
/// applies an explicit friction ellipse to the two Magic Formula outputs.
/// This provides a static-friction fallback at zero slip through the returned
/// peak values while preserving a finite, sign-correct response at very low
/// speed and malformed inputs.
pub fn calculate_pacejka_forces(
    slip_ratio: f64,
    slip_angle: f64,
    wheel_load: f64,
    available_grip: f64,
) -> PacejkaForceResult {
    calculate_pacejka_forces_with_parameters(
        &PacejkaParameters::default(),
        slip_ratio,
        slip_angle,
        wheel_load,
        available_grip,
    )
}

/// Parameterized form of [`calculate_pacejka_forces`] for terrain/compound
/// calibration without adding per-step allocations or mutable global state.
pub fn calculate_pacejka_forces_with_parameters(
    parameters: &PacejkaParameters,
    slip_ratio: f64,
    slip_angle: f64,
    wheel_load: f64,
    available_grip: f64,
) -> PacejkaForceResult {
    let load = safe_finite(wheel_load).max(0.0);
    let grip = safe_finite(available_grip).clamp(0.0, 2.0);
    if load <= 0.0 || grip <= 0.0 {
        return PacejkaForceResult {
            longitudinal: 0.0,
            lateral: 0.0,
            longitudinal_peak: 0.0,
            lateral_peak: 0.0,
            combined_magnitude: 0.0,
            combined_limit: 0.0,
        };
    }

    // Tires become less efficient as load rises. Keep the effect bounded so
    // unusually light/heavy authored vehicles remain controllable.
    let normalized_load = load / REFERENCE_TIRE_LOAD;
    let load_factor = (1.0 - 0.18 * (normalized_load - 1.0)).clamp(0.65, 1.15);
    let longitudinal_peak =
        load * grip * load_factor * safe_finite(parameters.longitudinal_d).max(0.0);
    let lateral_peak = load * grip * load_factor * safe_finite(parameters.lateral_d).max(0.0);
    let raw_longitudinal = longitudinal_peak
        * magic_formula(
            slip_ratio,
            parameters.longitudinal_b,
            parameters.longitudinal_c,
            1.0,
            parameters.longitudinal_e,
        );
    let raw_lateral = -lateral_peak
        * magic_formula(
            slip_angle,
            parameters.lateral_b,
            parameters.lateral_c,
            1.0,
            parameters.lateral_e,
        );

    // Ellipse normalization makes simultaneous braking/drive and cornering
    // consume one shared grip budget instead of allowing two independent
    // saturated force components.
    let normalized_longitudinal = raw_longitudinal / longitudinal_peak.max(1e-9);
    let normalized_lateral = raw_lateral / lateral_peak.max(1e-9);
    let demand = (normalized_longitudinal * normalized_longitudinal
        + normalized_lateral * normalized_lateral)
        .sqrt();
    let combined_scale = if demand > 1.0 { 1.0 / demand } else { 1.0 };
    let longitudinal = safe_finite(raw_longitudinal * combined_scale);
    let lateral = safe_finite(raw_lateral * combined_scale);
    let combined_magnitude = (longitudinal * longitudinal + lateral * lateral)
        .sqrt()
        .min(longitudinal_peak.max(lateral_peak));
    let combined_limit =
        (longitudinal_peak * longitudinal_peak + lateral_peak * lateral_peak).sqrt();

    PacejkaForceResult {
        longitudinal,
        lateral,
        longitudinal_peak,
        lateral_peak,
        combined_magnitude,
        combined_limit,
    }
}

/// Temperature-based grip factor.
fn temp_grip_factor(compound: TireCompound, temp: f64) -> f64 {
    let t_min = compound.optimal_temp_min();
    let t_max = compound.optimal_temp_max();
    if temp < t_min {
        // Cold: linear ramp from 0.5 at 0°C to 1.0 at optimal
        0.5 + 0.5 * (temp / t_min).clamp(0.0, 1.0)
    } else if temp <= t_max {
        1.0
    } else if temp <= 150.0 {
        // Overheating: linear drop
        1.0 - (temp - t_max) / (150.0 - t_max) * 0.6
    } else {
        0.3
    }
}

/// Pressure-based grip factor.
fn pressure_grip_factor(pressure: f64, nominal: f64) -> f64 {
    let ratio = pressure / nominal;
    if ratio < 0.5 {
        0.5 // flat
    } else if ratio < 0.7 {
        0.7
    } else if ratio > 1.5 {
        0.7 // overinflated
    } else {
        1.0
    }
}

/// Update a tire state for one simulation tick.
pub fn update_tire(
    tire: &mut TireState,
    thermal: &TireThermalParams,
    slip_ratio: f64,
    slip_angle: f64,
    load: f64,
    surface_roughness: f64,
    ambient_temp: f64,
    dt: f64,
) {
    if tire.is_flat {
        // Flat tire: minimal updates
        tire.grip_factor = 0.2;
        return;
    }

    let slip_energy = slip_ratio.abs() * load * tire.grip_factor;

    // Heat generation from slip
    let heat_in = thermal.heat_generation_rate * slip_energy * dt / thermal.thermal_mass;
    // Heat dissipation
    let heat_out = thermal.heat_dissipation_rate * (tire.temperature - ambient_temp) * dt
        / thermal.thermal_mass;

    tire.temperature += heat_in - heat_out;
    tire.temperature = tire.temperature.max(ambient_temp - 10.0);

    // Constant-volume ideal-gas approximation referenced to the authored
    // nominal pressure at 20 C. Both temperatures must be absolute.
    const NOMINAL_TEMPERATURE_K: f64 = 293.15;
    tire.pressure =
        tire.nominal_pressure * ((tire.temperature + 273.15) / NOMINAL_TEMPERATURE_K).max(0.0);

    // Wear from slip * load * roughness * compound wear rate
    let wear_rate = tire.compound.wear_rate();
    let slip_wear = (slip_ratio.abs() * 0.3 + slip_angle.abs() * 0.3).min(1.0);
    let load_factor = (load / 5000.0).clamp(0.5, 2.0);
    tire.wear += slip_wear * load_factor * surface_roughness * wear_rate * 0.0001 * dt;
    tire.wear = tire.wear.min(1.0);

    // Center vs shoulder wear
    let cornering_fraction = slip_angle.abs() / (slip_ratio.abs() + slip_angle.abs() + 1e-6);
    let center_increment = (1.0 - cornering_fraction)
        * slip_wear
        * load_factor
        * surface_roughness
        * wear_rate
        * 0.00005
        * dt;
    let shoulder_increment =
        cornering_fraction * slip_wear * load_factor * surface_roughness * wear_rate * 0.00005 * dt;
    tire.center_wear += center_increment;
    tire.shoulder_wear += shoulder_increment;

    // Flat spot from hard braking lockup
    if slip_ratio < -0.5 && load > 1000.0 {
        tire.center_wear += 0.001 * dt;
    }

    // Cord exposure
    tire.cord_exposed = tire.wear > 0.85;

    // Puncture risk increases with wear and rough surface
    if tire.wear > 0.7 && surface_roughness > 0.5 {
        let puncture_chance = (tire.wear - 0.7) * surface_roughness * 0.5 * dt;
        if !tire.has_puncture
            && rand_simple(tire.wear * 1000.0 + tire.temperature) < puncture_chance
        {
            tire.has_puncture = true;
            tire.wear = (tire.wear + 0.1).min(1.0);
        }
    }

    // Blowout risk
    tire.blowout_risk = ((tire.wear - 0.6) * 1.5
        + (tire.pressure / tire.nominal_pressure - 1.0).abs() * 0.5
        + if tire.temperature > 130.0 { 0.3 } else { 0.0 })
    .max(0.0);

    // Blistering from overheating
    if tire.temperature > 120.0 && !tire.has_blister {
        tire.has_blister = true;
        tire.grip_factor *= 0.9;
    }

    // Blowout
    if tire.blowout_risk > 1.0 && !tire.is_flat {
        tire.is_flat = true;
        tire.pressure = 0.0;
    }

    // Sidewall damage at extreme wear
    if tire.wear > 0.95 && !tire.is_flat {
        tire.is_flat = true;
    }

    // Compute grip factor
    let tg = temp_grip_factor(tire.compound, tire.temperature);
    let pg = pressure_grip_factor(tire.pressure, tire.nominal_pressure);
    let wear_penalty = 1.0 - tire.wear * 0.7;
    let flat_penalty = if tire.has_puncture { 0.5 } else { 1.0 };
    tire.grip_factor = tire.compound.base_grip() * tg * pg * wear_penalty * flat_penalty;
    tire.grip_factor = tire.grip_factor.clamp(0.0, 1.0);
}

/// Simple deterministic hash for pseudo-random decisions (no external dep).
fn rand_simple(seed: f64) -> f64 {
    let x = seed.sin() * 43758.5453;
    x - x.floor()
}

/// Get telemetry snapshot.
pub fn tire_telemetry(tire: &TireState) -> TireTelemetry {
    let failure = if tire.is_flat {
        if tire.blowout_risk > 1.0 {
            TireFailure::Blowout
        } else if tire.has_puncture {
            TireFailure::Puncture
        } else {
            TireFailure::FlatSpot
        }
    } else {
        TireFailure::None
    };
    TireTelemetry {
        temperature: tire.temperature,
        pressure: tire.pressure,
        wear: tire.wear,
        grip_factor: tire.grip_factor,
        is_flat: tire.is_flat,
        failure,
        center_wear: tire.center_wear,
        shoulder_wear: tire.shoulder_wear,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_tire() -> TireState {
        TireState::default()
    }

    #[test]
    fn test_heat_increases_with_slip() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        let t0 = tire.temperature;
        update_tire(&mut tire, &thermal, 0.5, 0.1, 5000.0, 0.3, 20.0, 1.0);
        assert!(tire.temperature > t0, "Slip should heat tire");
    }

    #[test]
    fn test_heat_dissipates() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        tire.temperature = 80.0;
        update_tire(&mut tire, &thermal, 0.0, 0.0, 0.0, 0.1, 20.0, 5.0);
        assert!(tire.temperature < 80.0, "Heat should dissipate");
    }

    #[test]
    fn test_temperature_affects_pressure() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        let p0 = tire.pressure;
        tire.temperature = 80.0;
        update_tire(&mut tire, &thermal, 0.3, 0.05, 5000.0, 0.3, 20.0, 0.1);
        // Pressure should change with temperature
        assert!(
            (tire.pressure - p0).abs() > 1.0,
            "Pressure should change with temp"
        );
    }

    #[test]
    fn test_pressure_uses_absolute_reference_temperature() {
        let thermal = TireThermalParams {
            heat_generation_rate: 0.0,
            heat_dissipation_rate: 0.0,
            thermal_mass: 8.0,
        };
        let mut tire = default_tire();
        tire.temperature = 40.0;
        update_tire(&mut tire, &thermal, 0.0, 0.0, 0.0, 0.0, 20.0, 0.1);
        let expected = tire.nominal_pressure * 313.15 / 293.15;
        assert!((tire.pressure - expected).abs() < 1e-9);
    }

    #[test]
    fn test_grip_optimal_at_compound_temperature() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        // Heat to optimal range
        tire.temperature = 80.0;
        update_tire(&mut tire, &thermal, 0.0, 0.0, 0.0, 0.1, 20.0, 0.1);
        let optimal_grip = tire.grip_factor;

        let mut cold_tire = default_tire();
        cold_tire.temperature = 5.0;
        update_tire(&mut cold_tire, &thermal, 0.0, 0.0, 0.0, 0.1, 20.0, 0.1);
        let cold_grip = cold_tire.grip_factor;

        assert!(
            optimal_grip > cold_grip,
            "Optimal temp should give better grip than cold"
        );
    }

    #[test]
    fn test_overheating_reduces_grip() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        tire.temperature = 60.0;
        update_tire(&mut tire, &thermal, 0.0, 0.0, 0.0, 0.1, 20.0, 0.1);
        let good_grip = tire.grip_factor;

        let mut hot_tire = default_tire();
        hot_tire.temperature = 60.0;
        hot_tire.compound = TireCompound::Sport;
        // Push to overheating
        for _ in 0..100 {
            update_tire(&mut hot_tire, &thermal, 0.8, 0.2, 5000.0, 0.5, 20.0, 0.1);
        }
        let hot_grip = hot_tire.grip_factor;
        assert!(
            good_grip > hot_grip || hot_tire.has_blister,
            "Overheating should reduce grip or blister"
        );
    }

    #[test]
    fn test_cold_tire_reduced_grip() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        tire.temperature = 5.0;
        update_tire(&mut tire, &thermal, 0.0, 0.0, 0.0, 0.1, 20.0, 0.1);
        assert!(
            tire.grip_factor < 0.85,
            "Cold tire should have reduced grip"
        );
    }

    #[test]
    fn test_wear_increases_with_slip() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        let w0 = tire.wear;
        for _ in 0..100 {
            update_tire(&mut tire, &thermal, 0.5, 0.3, 5000.0, 0.5, 20.0, 0.1);
        }
        assert!(tire.wear > w0, "Wear should increase with slip");
    }

    #[test]
    fn test_flat_spot_from_lockup() {
        // Use mild thermal params so tire doesn't blow out from heat
        let thermal = TireThermalParams {
            heat_generation_rate: 1.0,
            heat_dissipation_rate: 50.0,
            thermal_mass: 8.0,
        };
        let mut tire = default_tire();
        for _ in 0..500 {
            update_tire(&mut tire, &thermal, -0.8, 0.0, 5000.0, 0.5, 20.0, 0.01);
        }
        assert!(
            tire.center_wear > 0.0,
            "Lockup should cause center wear (flat spot)"
        );
    }

    #[test]
    fn test_puncture_chance_increases_with_wear() {
        // Verify puncture mechanism triggers by varying seed distribution
        let thermal = TireThermalParams {
            heat_generation_rate: 1.0,
            heat_dissipation_rate: 50.0,
            thermal_mass: 8.0,
        };
        let mut punctured = 0;
        for i in 0..500 {
            let mut tire = default_tire();
            tire.wear = 0.71 + (i as f64) * 0.0005;
            tire.temperature = 20.0 + (i as f64) * 0.5;
            for _ in 0..100 {
                let t = tire.temperature;
                update_tire(&mut tire, &thermal, 0.3, 0.1, 3000.0, 0.9, t, 0.01);
            }
            if tire.has_puncture {
                punctured += 1;
            }
        }
        assert!(
            punctured > 0,
            "High-wear tire on rough surface should sometimes puncture"
        );
    }

    #[test]
    fn test_blowout_at_high_risk() {
        let thermal = TireThermalParams::default();
        let mut tire = default_tire();
        tire.wear = 0.9;
        tire.pressure = 300.0; // overinflated
        for _ in 0..200 {
            update_tire(&mut tire, &thermal, 0.5, 0.3, 5000.0, 0.5, 20.0, 0.1);
        }
        // Should either be flat or have very high blowout risk
        assert!(
            tire.is_flat || tire.blowout_risk > 0.5,
            "Worn overinflated tire under stress should fail"
        );
    }

    #[test]
    fn test_tire_telemetry() {
        let mut tire = default_tire();
        tire.wear = 0.5;
        tire.temperature = 75.0;
        let telem = tire_telemetry(&tire);
        assert_eq!(telem.wear, 0.5);
        assert_eq!(telem.temperature, 75.0);
        assert_eq!(telem.failure, TireFailure::None);
    }

    #[test]
    fn test_damage_persistent() {
        let mut tire = default_tire();
        let thermal = TireThermalParams::default();
        // Wear it out
        for _ in 0..1000 {
            update_tire(&mut tire, &thermal, 0.5, 0.3, 5000.0, 0.5, 20.0, 0.1);
        }
        let wear_after_damage = tire.wear;
        // Stop driving
        for _ in 0..1000 {
            update_tire(&mut tire, &thermal, 0.0, 0.0, 0.0, 0.1, 20.0, 0.1);
        }
        assert!(
            tire.wear >= wear_after_damage,
            "Wear should not self-repair"
        );
    }

    #[test]
    fn magic_formula_lateral_response_has_a_peak_and_rolloff() {
        let peak = calculate_pacejka_forces(0.0, 0.12, 4000.0, 0.9);
        let gross_slip = calculate_pacejka_forces(0.0, 0.8, 4000.0, 0.9);

        assert!(
            peak.lateral.abs() > gross_slip.lateral.abs(),
            "lateral force should roll off after the peak: peak={}, gross={}",
            peak.lateral,
            gross_slip.lateral
        );
    }

    #[test]
    fn magic_formula_load_sensitivity_reduces_normalized_grip_at_high_load() {
        let light = calculate_pacejka_forces(0.0, 0.12, 2000.0, 0.9);
        let heavy = calculate_pacejka_forces(0.0, 0.12, 8000.0, 0.9);
        let light_mu = light.lateral.abs() / 2000.0;
        let heavy_mu = heavy.lateral.abs() / 8000.0;

        assert!(
            light_mu > heavy_mu,
            "normalized grip should decrease with load: light={light_mu}, heavy={heavy_mu}"
        );
    }

    #[test]
    fn magic_formula_combined_slip_reduces_both_force_components() {
        let pure_longitudinal = calculate_pacejka_forces(0.16, 0.0, 4000.0, 0.9);
        let pure_lateral = calculate_pacejka_forces(0.0, 0.12, 4000.0, 0.9);
        let combined = calculate_pacejka_forces(0.16, 0.12, 4000.0, 0.9);

        assert!(combined.longitudinal.abs() < pure_longitudinal.longitudinal.abs());
        assert!(combined.lateral.abs() < pure_lateral.lateral.abs());
        assert!(combined.combined_magnitude <= combined.combined_limit + 1e-9);
    }

    #[test]
    fn magic_formula_force_is_sign_symmetric_and_finite_at_low_speed() {
        let positive = calculate_pacejka_forces(0.1, 0.08, 1.0, 0.9);
        let negative = calculate_pacejka_forces(-0.1, -0.08, 1.0, 0.9);
        let invalid = calculate_pacejka_forces(f64::NAN, f64::INFINITY, 0.0, f64::NAN);

        assert!(positive.longitudinal > 0.0);
        assert!(positive.lateral < 0.0);
        assert!((positive.longitudinal + negative.longitudinal).abs() < 1e-9);
        assert!((positive.lateral + negative.lateral).abs() < 1e-9);
        assert!([
            invalid.longitudinal,
            invalid.lateral,
            invalid.combined_magnitude,
            invalid.combined_limit,
        ]
        .into_iter()
        .all(f64::is_finite));
    }
}
