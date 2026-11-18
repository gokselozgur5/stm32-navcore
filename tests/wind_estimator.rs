use core::f32::consts::PI;
use stm32_navcore::wind_estimator::*;

#[test]
fn test_wind_estimator_public_api_and_reexports() {
    let cfg = WindEstimatorConfig::default();
    let estimator = WindEstimator::new(cfg).unwrap();
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

#[test]
fn test_wind_estimator_input_validation_config() {
    let mut cfg = WindEstimatorConfig {
        time_constant_s: 0.0,
        ..Default::default()
    };
    assert_eq!(
        WindEstimator::new(cfg),
        Err(WindEstimatorError::InvalidTimeConstant)
    );

    cfg.time_constant_s = 10.0;
    cfg.min_airspeed_mps = -1.0;
    assert_eq!(
        WindEstimator::new(cfg),
        Err(WindEstimatorError::InvalidConfig)
    );

    cfg.min_airspeed_mps = 70.0;
    cfg.max_wind_speed_mps = 60.0;
    assert_eq!(
        WindEstimator::new(cfg),
        Err(WindEstimatorError::InvalidConfig)
    );
}

#[test]
fn test_wind_estimator_runtime_validation_errors() {
    let mut est = WindEstimator::new(WindEstimatorConfig::default()).unwrap();
    assert_eq!(
        est.update([20.0, 0.0], 20.0, 0.0, 0.0),
        Err(WindEstimatorError::InvalidSampleInterval)
    );
    assert_eq!(
        est.update([20.0, 0.0], 1.0, 0.0, 0.1),
        Err(WindEstimatorError::AirspeedTooLow)
    );
    assert_eq!(
        est.update([100.0, 0.0], 10.0, 0.0, 0.1),
        Err(WindEstimatorError::WindSpeedExceeded)
    );
}

#[test]
fn test_wind_estimator_tracking_convergence() {
    let cfg = WindEstimatorConfig {
        time_constant_s: 1.0,
        min_airspeed_mps: 5.0,
        max_wind_speed_mps: 50.0,
    };
    let mut est = WindEstimator::new(cfg).unwrap();
    // 5 m/s North, 5 m/s East wind
    // Aircraft heading North at 20 m/s TAS -> ground vel [25.0, 5.0]
    for _ in 0..100 {
        let _ = est.update([25.0, 5.0], 20.0, 0.0, 0.05).unwrap();
    }
    assert!((est.wind_vector()[0] - 5.0).abs() < 0.05);
    assert!((est.wind_vector()[1] - 5.0).abs() < 0.05);
}

#[test]
fn test_wind_estimator_reset_and_from_course() {
    let mut est = WindEstimator::new(WindEstimatorConfig::default()).unwrap();
    let res = est.update_from_course(25.0, 0.0, 20.0, 0.0, 0.1).unwrap();
    assert_eq!(est.sample_count(), 1);
    assert!((res.wind_ne_mps[0] - 5.0).abs() < 1e-4);

    est.reset();
    assert!(!est.is_initialized());
    assert_eq!(est.sample_count(), 0);
    assert_eq!(est.wind_vector(), [0.0, 0.0]);
}

#[test]
fn test_wind_correction_angle_and_bearing() {
    // Ground track 045 deg, aircraft heading 040 deg -> drift +5 deg
    let course = 45.0 * PI / 180.0;
    let heading = 40.0 * PI / 180.0;
    let wca = wind_correction_angle_rad(course, heading).unwrap();
    assert!((wca - (5.0 * PI / 180.0)).abs() < 1e-5);

    // Negative drift angle
    let wca_neg = wind_correction_angle_rad(heading, course).unwrap();
    assert!((wca_neg - (-5.0 * PI / 180.0)).abs() < 1e-5);
}
