use stm32_navcore::gyro_bias::{
    apply_bias_slew_limit, clamp_bias_vector, correct_gyro_sample, track_bias_from_error,
    track_bias_stationary, GyroBiasConfig, GyroBiasError, GyroBiasTracker,
};

#[test]
fn test_config_validation_and_errors() {
    let mut cfg = GyroBiasConfig::default();
    assert!(cfg.validate().is_ok());

    cfg.ki = -0.01;
    assert_eq!(cfg.validate(), Err(GyroBiasError::InvalidGain));
    cfg.ki = f32::NAN;
    assert_eq!(cfg.validate(), Err(GyroBiasError::InvalidGain));
    cfg.ki = 0.01;
    cfg.stationary_learning_rate = 0.0;
    assert_eq!(cfg.validate(), Err(GyroBiasError::InvalidGain));
    cfg.stationary_learning_rate = 1.5;
    assert_eq!(cfg.validate(), Err(GyroBiasError::InvalidGain));
    cfg.stationary_learning_rate = 0.05;

    cfg.max_bias_rad_s = -0.1;
    assert_eq!(cfg.validate(), Err(GyroBiasError::InvalidLimit));
    cfg.max_bias_rad_s = 0.1;

    cfg.max_drift_rate_rad_s2 = 0.0;
    assert_eq!(cfg.validate(), Err(GyroBiasError::InvalidLimit));
}

#[test]
fn test_correct_gyro_sample() {
    let raw = [0.15, -0.08, 0.02];
    let bias = [0.05, -0.03, 0.01];
    let corr = correct_gyro_sample(raw, bias).unwrap();
    assert!((corr[0] - 0.10).abs() < 1e-6);
    assert!((corr[1] - (-0.05)).abs() < 1e-6);
    assert!((corr[2] - 0.01).abs() < 1e-6);

    let nan_raw = [f32::NAN, 0.0, 0.0];
    assert_eq!(correct_gyro_sample(nan_raw, bias), Err(GyroBiasError::NonFinite));
}

#[test]
fn test_clamp_bias_vector() {
    let bias = [0.25, -0.30, 0.05];
    let clamped = clamp_bias_vector(bias, 0.1).unwrap();
    assert!((clamped[0] - 0.1).abs() < 1e-6);
    assert!((clamped[1] - (-0.1)).abs() < 1e-6);
    assert!((clamped[2] - 0.05).abs() < 1e-6);

    assert_eq!(clamp_bias_vector(bias, 0.0), Err(GyroBiasError::InvalidLimit));
    assert_eq!(clamp_bias_vector([f32::INFINITY, 0.0, 0.0], 0.1), Err(GyroBiasError::NonFinite));
}

#[test]
fn test_apply_bias_slew_limit() {
    let prev = [0.01, -0.01, 0.0];
    let candidate = [0.05, -0.05, 0.001];
    let max_rate = 0.01;
    let dt = 0.5; // max delta = 0.005

    let limited = apply_bias_slew_limit(candidate, prev, max_rate, dt).unwrap();
    assert!((limited[0] - 0.015).abs() < 1e-6);
    assert!((limited[1] - (-0.015)).abs() < 1e-6);
    assert!((limited[2] - 0.001).abs() < 1e-6);
}

#[test]
fn test_apply_bias_slew_limit_errors() {
    let prev = [0.0, 0.0, 0.0];
    let next = [0.01, 0.0, 0.0];
    assert_eq!(apply_bias_slew_limit(next, prev, 0.01, -0.1), Err(GyroBiasError::InvalidSampleInterval));
    assert_eq!(apply_bias_slew_limit(next, prev, 0.0, 0.1), Err(GyroBiasError::InvalidLimit));
    assert_eq!(apply_bias_slew_limit([f32::NAN, 0.0, 0.0], prev, 0.01, 0.1), Err(GyroBiasError::NonFinite));
}

#[test]
fn test_track_bias_from_error_integral() {
    let mut bias = [0.0f32; 3];
    let ki = 0.2f32;
    let dt = 0.1f32;
    let max_bias = 0.1f32;
    let error = [-0.02f32, 0.01f32, 0.0f32];
