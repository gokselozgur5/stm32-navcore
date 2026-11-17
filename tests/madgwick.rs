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
