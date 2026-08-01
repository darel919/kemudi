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
            heat_generation_rate: 200.0,
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

    // Pressure from ideal gas law approximation
    tire.pressure = tire.nominal_pressure * (1.0 + (tire.temperature - 20.0) / 273.15);

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

// === Body deformation ===

/// Impact location classification.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum ImpactLocation {
    Front,
    Rear,
    LeftSide,
    RightSide,
    Roof,
    Underbody,
    Wheel,
}

/// Impact event record.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpactEvent {
    pub severity: f64,
    pub direction: [f64; 3],
    pub location: ImpactLocation,
    pub timestamp: f64,
}

/// Per-beam damage state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeamDamage {
    pub original_length: f64,
    pub plastic_deformation: f64,
    pub yield_threshold: f64,
    pub elastic_limit: f64,
    pub is_permanently_deformed: bool,
}

/// Aggregate body damage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyDamage {
    pub total_impact_energy: f64,
    pub deformation_depth: f64,
    pub center_of_mass_offset: [f64; 3],
    pub broken_beams: Vec<usize>,
    pub damaged_beams: Vec<(usize, BeamDamage)>,
    pub alignment_offset: f64,
    pub active_warnings: Vec<String>,
    pub impact_history: Vec<ImpactEvent>,
}

impl Default for BodyDamage {
    fn default() -> Self {
        Self {
            total_impact_energy: 0.0,
            deformation_depth: 0.0,
            center_of_mass_offset: [0.0; 3],
            broken_beams: Vec::new(),
            damaged_beams: Vec::new(),
            alignment_offset: 0.0,
            active_warnings: Vec::new(),
            impact_history: Vec::new(),
        }
    }
}

/// Compute beam damage from impact force.
pub fn compute_beam_damage(beam_stiffness: f64, beam_length: f64, impact_force: f64) -> BeamDamage {
    let yield_threshold = beam_stiffness * beam_length * 0.01;
    let elastic_limit = yield_threshold * 0.8;
    let plastic_deformation = if impact_force > yield_threshold {
        (impact_force - yield_threshold) / beam_stiffness
    } else {
        0.0
    };
    BeamDamage {
        original_length: beam_length,
        plastic_deformation: plastic_deformation.min(beam_length * 0.3),
        yield_threshold,
        elastic_limit,
        is_permanently_deformed: plastic_deformation > 0.0,
    }
}

/// Assess impact and classify location.
pub fn assess_impact(
    velocity: f64,
    mass: f64,
    collision_normal: [f64; 3],
    _contact_node_z: f64,
    contact_node_y: f64,
    beam_stiffness: f64,
    beam_length: f64,
    timestamp: f64,
) -> (ImpactEvent, Vec<BeamDamage>) {
    let energy = 0.5 * mass * velocity * velocity;
    let severity = (energy / (mass * 100.0)).min(1.0);

    // Classify location from collision normal and contact point
    let location = if contact_node_y > 0.5 {
        ImpactLocation::Roof
    } else if collision_normal[2] < -0.5 {
        ImpactLocation::Front
    } else if collision_normal[2] > 0.5 {
        ImpactLocation::Rear
    } else if collision_normal[0] > 0.5 {
        ImpactLocation::RightSide
    } else if collision_normal[0] < -0.5 {
        ImpactLocation::LeftSide
    } else if contact_node_y < 0.1 {
        ImpactLocation::Underbody
    } else {
        ImpactLocation::Front
    };

    let event = ImpactEvent {
        severity,
        direction: collision_normal,
        location,
        timestamp,
    };

    // Compute damage to beams near impact
    let num_beams_affected = ((severity * 4.0) as usize).max(1);
    let mut damages = Vec::new();
    for i in 0..num_beams_affected {
        let impact_force = energy * (1.0 - i as f64 * 0.2) / (beam_length + 1.0);
        damages.push(compute_beam_damage(
            beam_stiffness,
            beam_length,
            impact_force,
        ));
    }

    (event, damages)
}

/// Compute center-of-mass offset from asymmetric beam damage.
pub fn deformation_affects_center_of_mass(
    damaged_beams: &[(usize, BeamDamage)],
    total_beams: usize,
) -> [f64; 3] {
    if damaged_beams.is_empty() || total_beams == 0 {
        return [0.0; 3];
    }
    let mut offset = [0.0f64; 3];
    for (_, bd) in damaged_beams {
        // Simplified: front beams (low index) push COM backward
        offset[2] -= bd.plastic_deformation * 0.1;
        offset[1] -= bd.plastic_deformation * 0.02;
    }
    let count = damaged_beams.len() as f64;
    [offset[0] / count, offset[1] / count, offset[2] / count]
}

/// Compute alignment offset (steering pull) from front beam damage.
pub fn deformation_affects_alignment(damaged_beams: &[(usize, BeamDamage)]) -> f64 {
    if damaged_beams.is_empty() {
        return 0.0;
    }
    let mut offset = 0.0;
    for (idx, bd) in damaged_beams {
        // Front beams (low indices) cause more alignment issue
        let front_factor = 1.0 - (*idx as f64 / 20.0).min(0.9);
        offset += bd.plastic_deformation * front_factor * 0.01;
    }
    offset
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

    // Body deformation tests

    #[test]
    fn test_beam_elastic_recovery() {
        let bd = compute_beam_damage(10000.0, 1.0, 50.0);
        assert_eq!(bd.plastic_deformation, 0.0);
        assert!(!bd.is_permanently_deformed);
    }

    #[test]
    fn test_beam_plastic_deformation() {
        let bd = compute_beam_damage(10000.0, 1.0, 500.0);
        assert!(bd.plastic_deformation > 0.0);
        assert!(bd.is_permanently_deformed);
    }

    #[test]
    fn test_impact_severity_scales_with_energy() {
        let (_, d1) = assess_impact(5.0, 1500.0, [0.0, 0.0, 1.0], 0.0, 0.0, 10000.0, 1.0, 0.0);
        let (_, d2) = assess_impact(20.0, 1500.0, [0.0, 0.0, 1.0], 0.0, 0.0, 10000.0, 1.0, 0.0);
        assert!(
            d2.len() >= d1.len(),
            "Higher energy should affect more beams"
        );
    }

    #[test]
    fn test_impact_classification_by_location() {
        // Vehicle assets point forward along -Z.
        let (event, _) = assess_impact(10.0, 1500.0, [0.0, 0.0, -1.0], 0.0, 0.0, 10000.0, 1.0, 0.0);
        assert_eq!(event.location, ImpactLocation::Front);

        let (event, _) = assess_impact(10.0, 1500.0, [0.0, 0.0, 1.0], 0.0, 0.0, 10000.0, 1.0, 0.0);
        assert_eq!(event.location, ImpactLocation::Rear);
    }

    #[test]
    fn test_center_of_mass_shift() {
        let damages = vec![
            (
                0,
                BeamDamage {
                    original_length: 1.0,
                    plastic_deformation: 0.1,
                    yield_threshold: 100.0,
                    elastic_limit: 80.0,
                    is_permanently_deformed: true,
                },
            ),
            (
                1,
                BeamDamage {
                    original_length: 1.0,
                    plastic_deformation: 0.05,
                    yield_threshold: 100.0,
                    elastic_limit: 80.0,
                    is_permanently_deformed: true,
                },
            ),
        ];
        let offset = deformation_affects_center_of_mass(&damages, 20);
        assert!(offset[2] != 0.0, "COM should shift from asymmetric damage");
    }

    #[test]
    fn test_alignment_from_front_damage() {
        let damages = vec![(
            0,
            BeamDamage {
                original_length: 1.0,
                plastic_deformation: 0.1,
                yield_threshold: 100.0,
                elastic_limit: 80.0,
                is_permanently_deformed: true,
            },
        )];
        let offset = deformation_affects_alignment(&damages);
        assert!(
            (offset - 0.0).abs() > 1e-6,
            "Front damage should affect alignment"
        );
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
}
