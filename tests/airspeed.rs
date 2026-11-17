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
