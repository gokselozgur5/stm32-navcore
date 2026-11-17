use stm32_navcore::mag_declination::{estimate_wmm2020, MagDeclinationError};

#[test]
fn test_wmm_invalid_inputs() {
    assert_eq!(
        estimate_wmm2020(f32::NAN, 0.0),
        Err(MagDeclinationError::NonFinite)
    );
    assert_eq!(
        estimate_wmm2020(0.0, f32::NAN),
        Err(MagDeclinationError::NonFinite)
    );
    assert_eq!(
        estimate_wmm2020(91.0, 0.0),
        Err(MagDeclinationError::InvalidLatitude)
    );
    assert_eq!(
        estimate_wmm2020(-91.0, 0.0),
        Err(MagDeclinationError::InvalidLatitude)
    );
    assert_eq!(
        estimate_wmm2020(0.0, 361.0),
        Err(MagDeclinationError::InvalidLongitude)
    );
}

#[test]
fn test_wmm_exact_grid_nodes() {
    let est = estimate_wmm2020(0.0, 0.0).unwrap();
    assert!((est.declination_deg - (-10.0)).abs() < 1e-4);
    assert!((est.inclination_deg - 6.0).abs() < 1e-4);

    let north = estimate_wmm2020(90.0, 0.0).unwrap();
    assert!((north.declination_deg - 180.0).abs() < 1e-4);
    assert!((north.inclination_deg - 85.0).abs() < 1e-4);
}
