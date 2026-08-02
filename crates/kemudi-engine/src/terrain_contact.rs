use serde::{Deserialize, Serialize};

use crate::tires::{calculate_pacejka_forces_with_parameters, PacejkaParameters};

/// Physical properties of a terrain surface material.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurfaceMaterial {
    pub id: u8,
    pub name: String,
    /// 0-1, how hard the surface is
    pub hardness: f64,
    /// 0-1, surface roughness
    pub roughness: f64,
    /// 0-1, base friction coefficient
    pub base_friction: f64,
    /// 0-1, rolling resistance factor
    pub rolling_resistance: f64,
    /// 0-1, deformability (mud/sand high, asphalt low)
    pub deformability: f64,
    /// 0-1, how much moisture affects grip
    pub moisture_factor: f64,
    /// 0-1, depth for soft surfaces
    pub depth: f64,
}

impl Default for SurfaceMaterial {
    fn default() -> Self {
        Self {
            id: 0,
            name: "Asphalt".to_string(),
            hardness: 0.95,
            roughness: 0.15,
            base_friction: 0.85,
            rolling_resistance: 0.015,
            deformability: 0.05,
            moisture_factor: 0.2,
            depth: 0.0,
        }
    }
}

/// 10 surface presets with realistic values.
pub fn surface_presets() -> Vec<SurfaceMaterial> {
    vec![
        SurfaceMaterial {
            id: 0,
            name: "Asphalt".to_string(),
            hardness: 0.95,
            roughness: 0.15,
            base_friction: 0.85,
            rolling_resistance: 0.015,
            deformability: 0.05,
            moisture_factor: 0.2,
            depth: 0.0,
        },
        SurfaceMaterial {
            id: 1,
            name: "Concrete".to_string(),
            hardness: 0.98,
            roughness: 0.2,
            base_friction: 0.8,
            rolling_resistance: 0.012,
            deformability: 0.02,
            moisture_factor: 0.15,
            depth: 0.0,
        },
        SurfaceMaterial {
            id: 2,
            name: "Gravel".to_string(),
            hardness: 0.5,
            roughness: 0.7,
            base_friction: 0.7,
            rolling_resistance: 0.035,
            deformability: 0.4,
            moisture_factor: 0.3,
            depth: 0.15,
        },
        SurfaceMaterial {
            id: 3,
            name: "Dirt".to_string(),
            hardness: 0.4,
            roughness: 0.5,
            base_friction: 0.6,
            rolling_resistance: 0.04,
            deformability: 0.5,
            moisture_factor: 0.45,
            depth: 0.2,
        },
        SurfaceMaterial {
            id: 4,
            name: "Grass".to_string(),
            hardness: 0.3,
            roughness: 0.4,
            base_friction: 0.5,
            rolling_resistance: 0.05,
            deformability: 0.45,
            moisture_factor: 0.4,
            depth: 0.1,
        },
        SurfaceMaterial {
            id: 5,
            name: "Sand".to_string(),
            hardness: 0.15,
            roughness: 0.6,
            base_friction: 0.4,
            rolling_resistance: 0.08,
            deformability: 0.8,
            moisture_factor: 0.25,
            depth: 0.5,
        },
        SurfaceMaterial {
            id: 6,
            name: "Mud".to_string(),
            hardness: 0.1,
            roughness: 0.5,
            base_friction: 0.3,
            rolling_resistance: 0.1,
            deformability: 0.9,
            moisture_factor: 0.7,
            depth: 0.6,
        },
        SurfaceMaterial {
            id: 7,
            name: "WetMud".to_string(),
            hardness: 0.08,
            roughness: 0.45,
            base_friction: 0.2,
            rolling_resistance: 0.12,
            deformability: 0.95,
            moisture_factor: 0.9,
            depth: 0.7,
        },
        SurfaceMaterial {
            id: 8,
            name: "Snow".to_string(),
            hardness: 0.2,
            roughness: 0.35,
            base_friction: 0.35,
            rolling_resistance: 0.06,
            deformability: 0.6,
            moisture_factor: 0.5,
            depth: 0.3,
        },
        SurfaceMaterial {
            id: 9,
            name: "Ice".to_string(),
            hardness: 0.9,
            roughness: 0.05,
            base_friction: 0.1,
            rolling_resistance: 0.008,
            deformability: 0.0,
            moisture_factor: 0.95,
            depth: 0.0,
        },
    ]
}

/// Current contact state between a wheel and terrain surface.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainContact {
    pub surface: SurfaceMaterial,
    /// Surface normal [nx, ny, nz]
    pub normal: [f64; 3],
    /// Slope angle in radians
    pub slope_angle: f64,
    /// 0-1, current moisture level
    pub moisture: f64,
    /// 0-1, surface compactness
    pub compactness: f64,
    /// Rut depth in meters
    pub rut_depth: f64,
}

impl Default for TerrainContact {
    fn default() -> Self {
        Self {
            surface: SurfaceMaterial::default(),
            normal: [0.0, 1.0, 0.0],
            slope_angle: 0.0,
            moisture: 0.0,
            compactness: 1.0,
            rut_depth: 0.0,
        }
    }
}

/// Result of traction calculation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TractionResult {
    pub friction_coefficient: f64,
    /// Peak terrain friction before the current slip state is evaluated.
    /// Tire state scales this value once, at the contact patch.
    pub available_friction: f64,
    pub longitudinal_grip: f64,
    pub lateral_grip: f64,
    pub rolling_resistance_force: f64,
    pub sinkage_depth: f64,
    pub is_valid: bool,
}

impl Default for TractionResult {
    fn default() -> Self {
        Self {
            friction_coefficient: 0.0,
            available_friction: 0.0,
            longitudinal_grip: 0.0,
            lateral_grip: 0.0,
            rolling_resistance_force: 0.0,
            sinkage_depth: 0.0,
            is_valid: false,
        }
    }
}

/// Calculate combined-slip traction for a wheel contacting a terrain surface.
///
/// Uses a simplified Pacejka-like friction ellipse:
/// - slip_ratio: (wheel_speed - ground_speed) / max(|ground_speed|, 1.0)
/// - slip_angle: atan2(lateral_velocity, max(|longitudinal_velocity|, 1.0))
/// - Surface-modified friction accounts for moisture, depth, deformability.
pub fn calculate_traction(
    contact: &TerrainContact,
    slip_ratio: f64,
    slip_angle: f64,
    wheel_load: f64,
    wheel_speed: f64,
) -> TractionResult {
    if !wheel_load.is_finite() || wheel_load <= 0.0 {
        return TractionResult::default();
    }

    let safe_slip_ratio = if slip_ratio.is_finite() {
        slip_ratio
    } else {
        0.0
    };
    let safe_slip_angle = if slip_angle.is_finite() {
        slip_angle
    } else {
        0.0
    };
    let safe_wheel_speed = if wheel_speed.is_finite() {
        wheel_speed.abs()
    } else {
        0.0
    };

    let s = &contact.surface;

    // Surface-modified base friction
    let moisture_dry = 1.0 - contact.moisture.clamp(0.0, 1.0) * s.moisture_factor.max(0.0) * 0.5;
    let depth_penalty = 1.0 - s.depth.clamp(0.0, 1.0) * 0.3;
    let compactness_factor = 0.5 + contact.compactness.clamp(0.0, 1.0) * 0.5;
    let effective_friction = (s.base_friction.max(0.0)
        * moisture_dry.max(0.0)
        * depth_penalty.max(0.0)
        * compactness_factor)
        .clamp(0.0, 2.0);

    let mut pacejka = PacejkaParameters::default();
    // Low-friction surfaces shed more force under gross slip. Retain the
    // authored Magic Formula shape while making ice/mud less forgiving than
    // dry asphalt at the same slip.
    let surface_ratio = (effective_friction / 0.85).clamp(0.0, 1.0);
    let curvature = 0.80 + 0.17 * surface_ratio;
    pacejka.longitudinal_e = curvature;
    pacejka.lateral_e = curvature;
    let tire_forces = calculate_pacejka_forces_with_parameters(
        &pacejka,
        safe_slip_ratio,
        safe_slip_angle,
        wheel_load,
        effective_friction,
    );
    let friction_coefficient = if wheel_load > 0.0 {
        tire_forces.combined_magnitude / wheel_load
    } else {
        0.0
    };
    let longitudinal_grip = if wheel_load > 0.0 {
        tire_forces.longitudinal.abs() / wheel_load
    } else {
        0.0
    };
    let lateral_grip = if wheel_load > 0.0 {
        tire_forces.lateral.abs() / wheel_load
    } else {
        0.0
    };

    // Rolling resistance
    let rolling_resistance_force = s.rolling_resistance.max(0.0) * wheel_load * compactness_factor;

    // Sinkage depth: increases with deformability, wheel speed, and softness
    let speed_sinkage = if s.deformability > 0.3 {
        // Mud/sand: sinkage increases with wheel speed
        safe_wheel_speed * s.deformability.max(0.0) * 0.005
    } else {
        0.0
    };
    let sinkage_depth = contact.rut_depth + speed_sinkage * (1.0 - s.hardness);

    TractionResult {
        friction_coefficient,
        available_friction: effective_friction,
        longitudinal_grip,
        lateral_grip,
        rolling_resistance_force,
        sinkage_depth,
        is_valid: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn asphalt() -> SurfaceMaterial {
        surface_presets()[0].clone()
    }

    fn ice() -> SurfaceMaterial {
        surface_presets()[9].clone()
    }

    fn mud() -> SurfaceMaterial {
        surface_presets()[6].clone()
    }

    fn sand() -> SurfaceMaterial {
        surface_presets()[5].clone()
    }

    fn default_contact(surface: SurfaceMaterial) -> TerrainContact {
        TerrainContact {
            surface,
            normal: [0.0, 1.0, 0.0],
            slope_angle: 0.0,
            moisture: 0.0,
            compactness: 1.0,
            rut_depth: 0.0,
        }
    }

    #[test]
    fn test_surface_preset_count() {
        assert_eq!(surface_presets().len(), 10);
    }

    #[test]
    fn test_surface_preset_friction_ranges() {
        for preset in &surface_presets() {
            assert!(
                preset.hardness >= 0.0 && preset.hardness <= 1.0,
                "{} hardness out of range",
                preset.name
            );
            assert!(
                preset.roughness >= 0.0 && preset.roughness <= 1.0,
                "{} roughness out of range",
                preset.name
            );
            assert!(
                preset.base_friction >= 0.0 && preset.base_friction <= 1.0,
                "{} base_friction out of range",
                preset.name
            );
            assert!(
                preset.rolling_resistance >= 0.0 && preset.rolling_resistance <= 1.0,
                "{} rolling_resistance out of range",
                preset.name
            );
            assert!(
                preset.deformability >= 0.0 && preset.deformability <= 1.0,
                "{} deformability out of range",
                preset.name
            );
            assert!(
                preset.moisture_factor >= 0.0 && preset.moisture_factor <= 1.0,
                "{} moisture_factor out of range",
                preset.name
            );
            assert!(
                preset.depth >= 0.0 && preset.depth <= 1.0,
                "{} depth out of range",
                preset.name
            );
        }
    }

    #[test]
    fn test_asphalt_friction_higher_than_ice() {
        let a = default_contact(asphalt());
        let i = default_contact(ice());
        let ra = calculate_traction(&a, 0.02, 0.0, 5000.0, 10.0);
        let ri = calculate_traction(&i, 0.02, 0.0, 5000.0, 10.0);
        assert!(
            ra.friction_coefficient > ri.friction_coefficient,
            "asphalt ({}) should have more grip than ice ({})",
            ra.friction_coefficient,
            ri.friction_coefficient
        );
    }

    #[test]
    fn test_asphalt_friction_higher_than_mud() {
        let a = default_contact(asphalt());
        let m = default_contact(mud());
        let ra = calculate_traction(&a, 0.02, 0.0, 5000.0, 10.0);
        let rm = calculate_traction(&m, 0.02, 0.0, 5000.0, 10.0);
        assert!(
            ra.friction_coefficient > rm.friction_coefficient,
            "asphalt ({}) should have more grip than mud ({})",
            ra.friction_coefficient,
            rm.friction_coefficient
        );
    }

    #[test]
    fn test_friction_stays_within_circle() {
        // friction_coefficient should not exceed the surface base_friction
        let contact = default_contact(asphalt());
        let result = calculate_traction(&contact, 0.5, 0.5, 5000.0, 10.0);
        assert!(
            result.friction_coefficient <= asphalt().base_friction * 1.01,
            "friction ({}) should not exceed base ({})",
            result.friction_coefficient,
            asphalt().base_friction
        );
    }

    #[test]
    fn test_gross_slip_is_less_forgiving_on_ice() {
        let asphalt_contact = default_contact(asphalt());
        let ice_contact = default_contact(ice());
        let asphalt_peak = calculate_traction(&asphalt_contact, 0.08, 0.0, 5000.0, 10.0);
        let asphalt_slide = calculate_traction(&asphalt_contact, 10.0, 0.0, 5000.0, 10.0);
        let ice_peak = calculate_traction(&ice_contact, 0.08, 0.0, 5000.0, 10.0);
        let ice_slide = calculate_traction(&ice_contact, 10.0, 0.0, 5000.0, 10.0);
        let asphalt_retention =
            asphalt_slide.friction_coefficient / asphalt_peak.friction_coefficient;
        let ice_retention = ice_slide.friction_coefficient / ice_peak.friction_coefficient;
        assert!(
            ice_retention < asphalt_retention,
            "ice should retain less of its peak grip at gross slip"
        );
    }

    #[test]
    fn test_mud_sinkage_increases_with_wheel_speed() {
        let contact = default_contact(mud());
        let r_slow = calculate_traction(&contact, 0.0, 0.0, 5000.0, 2.0);
        let r_fast = calculate_traction(&contact, 0.0, 0.0, 5000.0, 20.0);
        assert!(
            r_fast.sinkage_depth > r_slow.sinkage_depth,
            "fast sinkage ({}) should exceed slow ({})",
            r_fast.sinkage_depth,
            r_slow.sinkage_depth
        );
    }

    #[test]
    fn test_sand_sinkage_increases_with_speed() {
        let contact = default_contact(sand());
        let r_slow = calculate_traction(&contact, 0.0, 0.0, 5000.0, 2.0);
        let r_fast = calculate_traction(&contact, 0.0, 0.0, 5000.0, 20.0);
        assert!(
            r_fast.sinkage_depth > r_slow.sinkage_depth,
            "sand fast ({}) should exceed slow ({})",
            r_fast.sinkage_depth,
            r_slow.sinkage_depth
        );
    }

    #[test]
    fn test_wet_surface_reduces_grip() {
        let mut dry = default_contact(asphalt());
        dry.moisture = 0.0;
        let mut wet = default_contact(asphalt());
        wet.moisture = 0.8;
        let rd = calculate_traction(&dry, 0.02, 0.0, 5000.0, 10.0);
        let rw = calculate_traction(&wet, 0.02, 0.0, 5000.0, 10.0);
        assert!(
            rd.friction_coefficient > rw.friction_coefficient,
            "dry ({}) should have more grip than wet ({})",
            rd.friction_coefficient,
            rw.friction_coefficient
        );
    }

    #[test]
    fn test_zero_load_returns_invalid() {
        let contact = default_contact(asphalt());
        let result = calculate_traction(&contact, 0.0, 0.0, 0.0, 10.0);
        assert!(!result.is_valid);
    }

    #[test]
    fn test_rolling_resistance_positive() {
        let contact = default_contact(asphalt());
        let result = calculate_traction(&contact, 0.0, 0.0, 5000.0, 10.0);
        assert!(result.rolling_resistance_force > 0.0);
    }

    #[test]
    fn test_terrain_normal_affects_slope_angle() {
        // Slope angle of 45 degrees (PI/4)
        let contact = TerrainContact {
            surface: asphalt(),
            normal: [0.0, 0.707, 0.707],
            slope_angle: std::f64::consts::FRAC_PI_4,
            moisture: 0.0,
            compactness: 1.0,
            rut_depth: 0.0,
        };
        let result = calculate_traction(&contact, 0.0, 0.0, 5000.0, 10.0);
        assert!(result.is_valid);
    }

    #[test]
    fn test_low_compactness_reduces_grip() {
        let mut hard = default_contact(asphalt());
        hard.compactness = 1.0;
        let mut soft = default_contact(asphalt());
        soft.compactness = 0.3;
        let rh = calculate_traction(&hard, 0.02, 0.0, 5000.0, 10.0);
        let rs = calculate_traction(&soft, 0.02, 0.0, 5000.0, 10.0);
        assert!(
            rh.friction_coefficient > rs.friction_coefficient,
            "hard compact ({}) should exceed soft ({})",
            rh.friction_coefficient,
            rs.friction_coefficient
        );
    }
}
