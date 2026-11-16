use stm32_navcore::ahrs::{madgwick_ahrs_6dof_step, AhrsError, MadgwickAhrs};
use stm32_navcore::math::Quaternion;

#[test]
fn test_ahrs_initialization_and_reset() {
    let mut ahrs = MadgwickAhrs::default();
    assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
    assert!((ahrs.beta - MadgwickAhrs::DEFAULT_BETA).abs() < 1e-6);

    let custom = MadgwickAhrs::new(0.05);
    assert!((custom.beta - 0.05).abs() < 1e-6);

    let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let with_q = MadgwickAhrs::with_attitude(0.05, q).unwrap();
    assert_eq!(with_q.quaternion(), q);

    ahrs.reset();
    assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
}

#[test]
fn test_ahrs_invalid_inputs_rejected() {
    let mut ahrs = MadgwickAhrs::default();
    let g_ok = [0.0, 0.0, 0.0];
    let a_ok = [0.0, 0.0, 9.81];

    assert_eq!(ahrs.update(g_ok, a_ok, 0.0), Err(AhrsError::InvalidDt));
    assert_eq!(ahrs.update(g_ok, a_ok, -0.01), Err(AhrsError::InvalidDt));
    assert_eq!(ahrs.update(g_ok, a_ok, f32::NAN), Err(AhrsError::InvalidDt));

    let g_nan = [f32::NAN, 0.0, 0.0];
    assert_eq!(ahrs.update(g_nan, a_ok, 0.01), Err(AhrsError::NonFinite));

    let a_inf = [0.0, f32::INFINITY, 9.81];
    assert_eq!(ahrs.update(g_ok, a_inf, 0.01), Err(AhrsError::NonFinite));
}

#[test]
fn test_ahrs_pure_gyro_integration() {
    let mut ahrs = MadgwickAhrs::new(0.0);
    let dt = 0.01;
    let roll_rate = 0.5;

    for _ in 0..100 {
        ahrs.update([roll_rate, 0.0, 0.0], [0.0, 0.0, 0.0], dt)
            .unwrap();
    }
    let (roll, pitch, yaw) = ahrs.euler_zyx();
    assert!((roll - 0.5).abs() < 0.02);
    assert!(pitch.abs() < 1e-3);
    assert!(yaw.abs() < 1e-3);
}
