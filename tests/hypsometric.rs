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
