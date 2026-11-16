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
