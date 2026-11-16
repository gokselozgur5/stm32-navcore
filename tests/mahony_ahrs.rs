use stm32_navcore::mahony_ahrs::{
    mahony_ahrs_6dof_step, mahony_ahrs_9dof_step, MahonyAhrs, MahonyAhrsError,
};
use stm32_navcore::math::Quaternion;

#[test]
fn test_mahony_initialization_and_reset() {
    let mut ahrs = MahonyAhrs::default();
    assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
    assert_eq!(ahrs.gyro_bias(), [0.0; 3]);
    assert!((ahrs.kp - MahonyAhrs::DEFAULT_KP).abs() < 1e-6);

    let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let with_q = MahonyAhrs::with_attitude(2.0, 0.1, q).unwrap();
    assert_eq!(with_q.quaternion(), q);

    ahrs.set_gyro_bias([0.01, -0.02, 0.03]);
    assert_eq!(ahrs.gyro_bias(), [0.01, -0.02, 0.03]);
    ahrs.reset_gyro_bias();
    assert_eq!(ahrs.gyro_bias(), [0.0; 3]);
}

#[test]
fn test_mahony_invalid_dt_rejected() {
    let mut ahrs = MahonyAhrs::default();
    let g_ok = [0.0, 0.0, 0.0];
    let a_ok = [0.0, 0.0, 9.81];

    assert_eq!(ahrs.update(g_ok, a_ok, 0.0), Err(MahonyAhrsError::InvalidDt));
    assert_eq!(ahrs.update(g_ok, a_ok, -0.01), Err(MahonyAhrsError::InvalidDt));
    assert_eq!(ahrs.update(g_ok, a_ok, f32::NAN), Err(MahonyAhrsError::InvalidDt));
}

#[test]
fn test_mahony_non_finite_inputs_rejected() {
    let mut ahrs = MahonyAhrs::default();
    let g_ok = [0.0, 0.0, 0.0];
    let a_ok = [0.0, 0.0, 9.81];

    let g_nan = [f32::NAN, 0.0, 0.0];
    assert_eq!(ahrs.update(g_nan, a_ok, 0.01), Err(MahonyAhrsError::NonFinite));

    let a_inf = [0.0, f32::INFINITY, 9.81];
    assert_eq!(ahrs.update(g_ok, a_inf, 0.01), Err(MahonyAhrsError::NonFinite));
}
