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
