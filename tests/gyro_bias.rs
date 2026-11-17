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
