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

#[test]
fn test_damp_velocity_step_decay() {
    use stm32_navcore::vel_damping::damp_velocity_step;

    let v0 = [10.0, -20.0, 5.0];
    let tau = 100.0;
    let dt = 1.0;
    let v1 = damp_velocity_step(v0, tau, dt).unwrap();

    let expected_decay = libm::expf(-1.0 / 100.0);
    assert!((v1[0] - v0[0] * expected_decay).abs() < 1e-6);
    assert!((v1[1] - v0[1] * expected_decay).abs() < 1e-6);
    assert!((v1[2] - v0[2] * expected_decay).abs() < 1e-6);

    let err_dt = damp_velocity_step(v0, tau, -0.1);
    assert_eq!(err_dt, Err(VelDampingError::InvalidSampleInterval));

    let err_tau = damp_velocity_step(v0, 0.0, dt);
    assert_eq!(err_tau, Err(VelDampingError::InvalidTimeConstant));
}

#[test]
fn test_innovation_and_gate_outlier_rejection() {
    use stm32_navcore::vel_damping::{calculate_innovation, fuse_velocity_step};

    let v_dr = [10.0, 0.0, 0.0];
    let v_gps = [13.0, 4.0, 0.0];
    let (innov, norm) = calculate_innovation(v_dr, v_gps).unwrap();
    assert_eq!(innov, [3.0, 4.0, 0.0]);
    assert!((norm - 5.0).abs() < 1e-6);

    let res_ok = fuse_velocity_step(v_dr, v_gps, 0.5, 6.0);
    assert_eq!(res_ok, Ok([11.5, 2.0, 0.0]));

    let res_exceeded = fuse_velocity_step(v_dr, v_gps, 0.5, 4.0);
    assert_eq!(res_exceeded, Err(VelDampingError::InnovationGateExceeded));
}
