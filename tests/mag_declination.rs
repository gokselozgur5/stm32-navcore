use stm32_navcore::mag_declination::{
    estimate_wmm2020, estimate_wmm2020_rad, magnetic_to_true_heading_rad,
    true_to_magnetic_heading_rad, MagDeclinationError,
};

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

#[test]
fn test_wmm_bilinear_interpolation_midpoint() {
    let mid = estimate_wmm2020(15.0, 15.0).unwrap();
    assert!((mid.declination_deg - (-4.5)).abs() < 1e-3);
    assert!((mid.inclination_deg - 24.0).abs() < 1e-3);

    let rad = estimate_wmm2020_rad(0.0, 0.0).unwrap();
    let expected_rad = -10.0 * core::f32::consts::PI / 180.0;
    assert!((rad.declination_rad() - expected_rad).abs() < 1e-4);
}

#[test]
fn test_heading_conversions() {
    let pi = core::f32::consts::PI;
    let true_hdg = magnetic_to_true_heading_rad(0.5, 0.1);
    assert!((true_hdg - 0.6).abs() < 1e-5);
    let mag_hdg = true_to_magnetic_heading_rad(true_hdg, 0.1);
    assert!((mag_hdg - 0.5).abs() < 1e-5);

    let wrap_true = magnetic_to_true_heading_rad(2.0 * pi - 0.1, 0.2);
    assert!((wrap_true - 0.1).abs() < 1e-5);
}

#[test]
fn normalize_lon_deg_large_input_terminates() {
    use std::sync::mpsc::channel;
    use std::time::Duration;
    use stm32_navcore::mag_declination::MagDeclinationLut;

    let (tx, rx) = channel();
    std::thread::spawn(move || {
        let _ = tx.send(MagDeclinationLut::normalize_lon_deg(1.0e20));
    });
    let done = rx.recv_timeout(Duration::from_millis(500));
    assert!(done.is_ok(), "normalize_lon_deg(1.0e20) did not return");
}

