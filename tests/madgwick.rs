use stm32_navcore::madgwick::{
    madgwick_imu_step, madgwick_marg_step, MadgwickAhrs, MadgwickAhrsError,
};
use stm32_navcore::math::Quaternion;

#[test]
fn test_madgwick_init_and_getters() {
    let mut ahrs = MadgwickAhrs::default();
    assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
    assert!((ahrs.beta() - MadgwickAhrs::DEFAULT_BETA_IMU).abs() < 1e-6);

    assert!(ahrs.set_beta(0.2).is_ok());
    assert_eq!(ahrs.beta(), 0.2);
    assert_eq!(ahrs.set_beta(0.0), Err(MadgwickAhrsError::InvalidBeta));
    assert_eq!(ahrs.set_beta(-1.0), Err(MadgwickAhrsError::InvalidBeta));

    ahrs.reset();
    assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
}

#[test]
fn test_madgwick_with_attitude_and_euler() {
    let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let filter = MadgwickAhrs::with_attitude(0.05, q).unwrap();
    assert_eq!(filter.quaternion(), q);

    let (roll, pitch, yaw) = filter.euler_zyx();
    assert!((roll.abs() - core::f32::consts::PI).abs() < 1e-4);
    assert!(pitch.abs() < 1e-4);
    assert!(yaw.abs() < 1e-4);

    let zero_q = Quaternion::new(0.0, 0.0, 0.0, 0.0);
    assert_eq!(
        MadgwickAhrs::with_attitude(0.05, zero_q),
        Err(MadgwickAhrsError::NormalizationFailed)
    );
}

#[test]
fn test_madgwick_validation_errors() {
    let mut ahrs = MadgwickAhrs::default();
    let g = [0.0, 0.0, 0.0];
    let a = [0.0, 0.0, 9.81];
    let m = [0.2, 0.0, 0.4];

    assert_eq!(
        ahrs.update_imu(g, a, 0.0),
        Err(MadgwickAhrsError::InvalidDt)
    );
    assert_eq!(
        ahrs.update_imu(g, a, -0.01),
        Err(MadgwickAhrsError::InvalidDt)
    );
    assert_eq!(
        ahrs.update_imu([f32::NAN, 0.0, 0.0], a, 0.01),
        Err(MadgwickAhrsError::NonFinite)
    );
    assert_eq!(
        ahrs.update_imu(g, [0.0, f32::INFINITY, 0.0], 0.01),
        Err(MadgwickAhrsError::NonFinite)
    );
    assert_eq!(
        ahrs.update_marg(g, a, [f32::NAN, 0.0, 0.0], 0.01),
        Err(MadgwickAhrsError::NonFinite)
    );
    assert!(ahrs.update_marg(g, a, m, 0.01).is_ok());
}

#[test]
fn test_madgwick_pure_gyro_integration() {
    let mut ahrs = MadgwickAhrs::new(0.001).unwrap();
    let dt = 0.01;
    let rate_z = 1.0;
    for _ in 0..100 {
        ahrs.update_imu([0.0, 0.0, rate_z], [0.0, 0.0, 0.0], dt)
            .unwrap();
    }
    let (_roll, _pitch, yaw) = ahrs.euler_zyx();
    assert!((yaw - 1.0).abs() < 0.05);
}

#[test]
fn test_madgwick_imu_convergence() {
    let q_init = Quaternion::new(0.92388, 0.38268, 0.0, 0.0);
    let mut ahrs = MadgwickAhrs::with_attitude(0.5, q_init).unwrap();
    let dt = 0.02;
    let accel = [0.0, 0.0, -9.81];
    for _ in 0..300 {
        ahrs.update_imu([0.0, 0.0, 0.0], accel, dt).unwrap();
    }
    let (roll, pitch, _yaw) = ahrs.euler_zyx();
    assert!(roll.abs() < 0.05);
    assert!(pitch.abs() < 0.05);
}

#[test]
fn test_madgwick_marg_heading_tracking() {
    let mut ahrs = MadgwickAhrs::new(0.5).unwrap();
    let dt = 0.02;
    let accel = [0.0, 0.0, -9.81];
    let mag = [0.2, 0.0, 0.4];
    for _ in 0..300 {
        ahrs.update_marg([0.0, 0.0, 0.0], accel, mag, dt).unwrap();
    }
    let (roll, pitch, yaw) = ahrs.euler_zyx();
    assert!(roll.abs() < 0.05);
    assert!(pitch.abs() < 0.05);
    assert!(yaw.abs() < 0.05);
}

#[test]
fn test_madgwick_free_step_functions() {
    let q = Quaternion::IDENTITY;
    let g = [0.0, 0.0, 0.0];
    let a = [0.0, 0.0, 9.81];
    let m = [0.2, 0.0, 0.4];

    let q_imu = madgwick_imu_step(q, g, a, 0.1, 0.01).unwrap();
    assert!((q_imu.norm() - 1.0).abs() < 1e-6);

    let q_marg = madgwick_marg_step(q, g, a, m, 0.1, 0.01).unwrap();
    assert!((q_marg.norm() - 1.0).abs() < 1e-6);
}
