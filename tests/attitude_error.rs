use stm32_navcore::attitude::{attitude_error, attitude_error_angle};
use stm32_navcore::math::Quaternion;

#[test]
fn test_attitude_error_identity() {
    let q = Quaternion::IDENTITY;
    let err = attitude_error(&q, &q).unwrap();
    assert_eq!(err.angle_rad, 0.0);
    assert_eq!(err.axis, [0.0, 0.0, 0.0]);
    assert_eq!(attitude_error_angle(&q, &q).unwrap(), 0.0);
}
