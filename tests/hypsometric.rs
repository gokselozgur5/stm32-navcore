use stm32_navcore::hypsometric::{
    hypsometric_altitude, hypsometric_thickness, HypsometricError, HypsometricProfile,
    TempLapseCompensator, STANDARD_SEA_LEVEL_PRESSURE_PA, STANDARD_SEA_LEVEL_TEMP_K,
};

#[test]
fn test_integration_hypsometric_thickness() {
    let p_sea = STANDARD_SEA_LEVEL_PRESSURE_PA;
    let p_1km = 89874.6;
    let t_avg = 284.9;
    let thickness = hypsometric_thickness(p_sea, p_1km, t_avg).unwrap();
    assert!((thickness - 1000.0).abs() < 25.0);
}

#[test]
fn test_integration_hypsometric_profile_reversibility() {
    let profile = HypsometricProfile::standard_isa();
    for &alt in &[0.0, 500.0, 1500.0, 3000.0, 8000.0] {
        let p = profile.pressure_at_altitude(alt).unwrap();
        let recovered = profile.altitude_at_pressure(p).unwrap();
        assert!((recovered - alt).abs() < 0.2);
    }
}

#[test]
fn test_integration_temp_lapse_compensation() {
    let comp = TempLapseCompensator::standard();
    let indicated_alt = 2000.0;
    let oat_hot = 300.0;
    let oat_cold = 250.0;
    let true_alt_hot = comp
        .true_altitude_from_indicated(indicated_alt, oat_hot)
        .unwrap();
    let true_alt_cold = comp
        .true_altitude_from_indicated(indicated_alt, oat_cold)
        .unwrap();
    assert!(true_alt_hot > indicated_alt);
    assert!(true_alt_cold < indicated_alt);
}

#[test]
fn test_integration_density_altitude_and_errors() {
    let comp = TempLapseCompensator::standard();
    let da = comp
        .density_altitude(
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            STANDARD_SEA_LEVEL_TEMP_K + 10.0,
        )
        .unwrap();
    assert!((da - 365.76).abs() < 1.0, "da at +10 K = {da}");

    assert_eq!(
        comp.density_altitude(0.0, 288.15),
        Err(HypsometricError::InvalidPressure)
    );
}

#[test]
fn test_integration_hypsometric_altitude_free_function() {
    let alt = hypsometric_altitude(
        STANDARD_SEA_LEVEL_PRESSURE_PA,
        STANDARD_SEA_LEVEL_PRESSURE_PA,
        STANDARD_SEA_LEVEL_TEMP_K,
        0.0065,
    )
    .unwrap();
    assert!(alt.abs() < 1e-3);
}
