use stm32_navcore::geo::mag_declination::{
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
    assert!((est.declination_deg - (-4.6682)).abs() < 1e-3);
    assert!((est.inclination_deg - 6.0).abs() < 1e-4);

    let north = estimate_wmm2020(90.0, 0.0).unwrap();
    assert!((north.declination_deg - 3.6569).abs() < 1e-3);
    assert!((north.inclination_deg - 85.0).abs() < 1e-4);
}

#[test]
fn test_wmm_bilinear_interpolation_midpoint() {
    let mid = estimate_wmm2020(15.0, 15.0).unwrap();
    assert!((mid.declination_deg - 1.7065).abs() < 1e-3);
    assert!((mid.inclination_deg - 24.0).abs() < 1e-3);

    let rad = estimate_wmm2020_rad(0.0, 0.0).unwrap();
    let expected_rad = -4.6681676 * core::f32::consts::PI / 180.0;
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
    use stm32_navcore::geo::mag_declination::MagDeclinationLut;

    let (tx, rx) = channel();
    std::thread::spawn(move || {
        let _ = tx.send(MagDeclinationLut::normalize_lon_deg(1.0e20));
    });
    let done = rx.recv_timeout(Duration::from_millis(500));
    assert!(done.is_ok(), "normalize_lon_deg(1.0e20) did not return");
}

#[test]
fn test_wmm_southern_ocean_points_match_harmonic_model() {
    use stm32_navcore::geo::wmm_declination::wmm_declination_deg;

    let points = [(-60.0, 90.0), (-60.0, 60.0), (-60.0, 120.0)];
    for &(lat, lon) in &points {
        let est = estimate_wmm2020(lat, lon).unwrap();
        let expected = wmm_declination_deg(lat, lon).unwrap();
        assert!(
            (est.declination_deg - expected).abs() < 1e-4,
            "mismatch at lat={lat}, lon={lon}: est={}, expected={}",
            est.declination_deg,
            expected
        );
    }
}

#[test]
fn test_wmm_custom_lut_antimeridian_seam_wrap() {
    use stm32_navcore::geo::mag_declination::{MagDeclinationLut, WMM_LAT_ROWS, WMM_LON_COLS};

    let mut dec_grid = [[0.0f32; WMM_LON_COLS]; WMM_LAT_ROWS];
    let inc_grid = [[0.0f32; WMM_LON_COLS]; WMM_LAT_ROWS];
    for r in 0..WMM_LAT_ROWS {
        for c in 0..WMM_LON_COLS {
            dec_grid[r][c] = if c % 2 == 0 { 175.0 } else { -175.0 };
        }
    }
    let lut = MagDeclinationLut::new(dec_grid, inc_grid);
    let est = lut.estimate(0.0, -165.0).unwrap();
    assert!(
        (est.declination_deg - 180.0).abs() < 1e-2,
        "expected ~180.0 seam wrap, got {}",
        est.declination_deg
    );
    assert!(
        est.declination_deg.abs() > 90.0,
        "must wrap through antimeridian, never 0.0"
    );
}

#[test]
fn test_wmm_seam_continuity_antimeridian() {
    let east = estimate_wmm2020(10.0, 180.0).unwrap();
    let west = estimate_wmm2020(10.0, -180.0).unwrap();
    assert!(
        (east.declination_deg - west.declination_deg).abs() < 1e-3,
        "antimeridian declination discontinuity: east={}, west={}",
        east.declination_deg,
        west.declination_deg
    );
}
