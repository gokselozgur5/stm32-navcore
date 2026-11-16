use stm32_navcore::vel_damping::{VelDampingConfig, VelDampingError};

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

#[test]
fn test_gps_fusion_convergence() {
    use stm32_navcore::vel_damping::VelDampingFilter;

    let cfg = VelDampingConfig {
        tau_s: 1000.0,
        gps_gain: 0.2,
        max_innovation_mps: 20.0,
        gps_timeout_s: 2.0,
    };
    let mut filter = VelDampingFilter::new(cfg).unwrap();
    let gps_target = [10.0, 5.0, -2.0];

    for _ in 0..80 {
        let _ = filter.fuse_gps(gps_target).unwrap();
    }
    let v = filter.velocity();
    assert!((v[0] - gps_target[0]).abs() < 1e-4);
    assert!((v[1] - gps_target[1]).abs() < 1e-4);
    assert!((v[2] - gps_target[2]).abs() < 1e-4);
    assert!(filter.is_gps_valid());
    assert_eq!(filter.time_since_gps(), 0.0);
}

#[test]
fn test_dead_reckoning_outage_propagation() {
    use stm32_navcore::vel_damping::VelDampingFilter;

    let cfg = VelDampingConfig {
        tau_s: 100.0,
        gps_gain: 0.5,
        max_innovation_mps: 10.0,
        gps_timeout_s: 1.0,
    };
    let mut filter = VelDampingFilter::new(cfg).unwrap();
    let _ = filter.fuse_gps([10.0, 0.0, 0.0]).unwrap();
    assert!(filter.is_gps_valid());

    let _ = filter.predict([1.0, 0.0, 0.0], 0.5).unwrap();
    assert!(filter.is_gps_valid());

    let _ = filter.predict([1.0, 0.0, 0.0], 0.6).unwrap();
    assert!(!filter.is_gps_valid());
    assert!((filter.time_since_gps() - 1.1).abs() < 1e-6);
}

#[test]
fn test_step_and_non_finite_rejection() {
    use stm32_navcore::vel_damping::VelDampingFilter;

    let cfg = VelDampingConfig::default();
    let mut filter = VelDampingFilter::new(cfg).unwrap();

    let err_accel = filter.predict([f32::NAN, 0.0, 0.0], 0.1);
    assert_eq!(err_accel, Err(VelDampingError::NonFinite));

    let err_fuse = filter.fuse_gps([0.0, f32::INFINITY, 0.0]);
    assert_eq!(err_fuse, Err(VelDampingError::NonFinite));

    let res = filter.step([1.0, 0.0, 0.0], Some([0.1, 0.0, 0.0]), 0.1);
    assert!(res.is_ok());

    assert!(filter.reset_velocity([5.0, -5.0, 1.0]).is_ok());
    assert_eq!(filter.velocity(), [5.0, -5.0, 1.0]);
}
