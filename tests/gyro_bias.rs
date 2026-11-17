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
