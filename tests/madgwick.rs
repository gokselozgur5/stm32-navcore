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
