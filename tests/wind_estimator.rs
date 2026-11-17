use core::f32::consts::PI;
use stm32_navcore::wind_estimator::*;

#[test]
fn test_wind_estimator_public_api_and_reexports() {
    let cfg = WindEstimatorConfig::default();
    let mut estimator = WindEstimator::new(cfg).unwrap();
    assert!(!estimator.is_initialized());
    assert_eq!(estimator.sample_count(), 0);
    assert_eq!(estimator.wind_vector(), [0.0, 0.0]);
    assert_eq!(estimator.wind_speed(), 0.0);
}

#[test]
fn test_wind_triangle_geometry_headwind_tailwind() {
    // Aircraft heading North (psi = 0) at 30 m/s TAS
    // Pure tailwind of 10 m/s (blowing North): ground vel [40.0, 0.0]
    let w_vec = instantaneous_wind_vector([40.0, 0.0], 30.0, 0.0).unwrap();
    assert!((w_vec[0] - 10.0).abs() < 1e-5);
    assert!(w_vec[1].abs() < 1e-5);

    let rel = decompose_relative_wind(w_vec, 0.0).unwrap();
    assert!((rel.headwind_mps - (-10.0)).abs() < 1e-5);
    assert!(rel.crosswind_mps.abs() < 1e-5);
}

#[test]
fn test_wind_triangle_geometry_crosswind() {
    // Heading North (psi = 0) at 25 m/s TAS
    // Wind blowing from East (90 deg) towards West at 8 m/s: [0.0, -8.0]
    // Ground speed: North 25 m/s, East -8.0 m/s
    let w_vec = instantaneous_wind_vector([25.0, -8.0], 25.0, 0.0).unwrap();
    assert!(w_vec[0].abs() < 1e-5);
    assert!((w_vec[1] - (-8.0)).abs() < 1e-5);

    let rel = decompose_relative_wind(w_vec, 0.0).unwrap();
    assert!(rel.headwind_mps.abs() < 1e-5);
    assert!((rel.crosswind_mps - 8.0).abs() < 1e-5);
}
