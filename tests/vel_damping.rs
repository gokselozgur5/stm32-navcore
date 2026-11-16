use stm32_navcore::vel_damping::{
    VelDampingConfig, VelDampingError,
};

#[test]
fn test_config_validation_and_errors() {
    let mut cfg = VelDampingConfig::default();
    assert!(cfg.validate().is_ok());

    cfg.tau_s = 0.0;
    assert_eq!(cfg.validate(), Err(VelDampingError::InvalidTimeConstant));

    cfg.tau_s = 60.0;
    cfg.gps_gain = -0.1;
    assert_eq!(cfg.validate(), Err(VelDampingError::InvalidGain));

    cfg.gps_gain = 1.5;
    assert_eq!(cfg.validate(), Err(VelDampingError::InvalidGain));

    cfg.gps_gain = 0.2;
    cfg.max_innovation_mps = -1.0;
    assert_eq!(cfg.validate(), Err(VelDampingError::InvalidGateThreshold));
}
