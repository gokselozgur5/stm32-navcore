use stm32_navcore::airspeed::*;

#[test]
fn test_standard_sea_level_density_and_sound() {
    let rho = air_density(STANDARD_PRESSURE_PA, STANDARD_TEMP_K).unwrap();
    assert!((rho - STANDARD_DENSITY_KG_M3).abs() < 1e-3);

    let a = speed_of_sound(STANDARD_TEMP_K).unwrap();
    assert!((a - 340.294).abs() < 0.1);
}

#[test]
fn test_incompressible_and_calibrated_airspeed() {
    let q = 841.4;
    let ias = incompressible_ias(q).unwrap();
    let cas = calibrated_airspeed(q).unwrap();
    assert!((ias - 37.06).abs() < 0.2);
    assert!((cas - 37.06).abs() < 0.2);
    assert!((ias - cas).abs() < 0.1);
}

#[test]
fn test_high_subsonic_compressibility() {
    let q = 15000.0;
    let ias = incompressible_ias(q).unwrap();
    let cas = calibrated_airspeed(q).unwrap();
    assert!(cas < ias);
    let mach = impact_pressure_to_mach(q, STANDARD_PRESSURE_PA).unwrap();
    assert!(mach > 0.4 && mach < 0.5);
}

#[test]
fn test_true_airspeed_at_altitude() {
    let p_stat = 54020.0;
    let t_k = 255.65;
    let q = 2000.0;
    let tas = true_airspeed(q, p_stat, t_k).unwrap();
    let ias = incompressible_ias(q).unwrap();
    assert!(tas > ias);
}

#[test]
fn test_pressure_and_density_altitude() {
    let p_alt_sea = pressure_altitude(STANDARD_PRESSURE_PA).unwrap();
    assert!(p_alt_sea.abs() < 0.5);

    let d_alt_sea = density_altitude(STANDARD_PRESSURE_PA, STANDARD_TEMP_K).unwrap();
    assert!(d_alt_sea.abs() < 1.0);
}

#[test]
fn test_calculator_struct_and_errors() {
    let calc = AirspeedCalculator::new();
    let state = calc
        .compute(1000.0, STANDARD_PRESSURE_PA, STANDARD_TEMP_K)
        .unwrap();
    assert!(state.tas_mps > 0.0);
    assert!(state.mach > 0.0);

    assert_eq!(
