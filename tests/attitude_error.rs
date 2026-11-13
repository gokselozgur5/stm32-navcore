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

#[test]
fn test_attitude_error_single_axis() {
    let q_id = Quaternion::IDENTITY;
    let angle = 0.42f32;
    let half = angle * 0.5;

    let q_x = Quaternion::new(libm::cosf(half), libm::sinf(half), 0.0, 0.0);
    let err_x = attitude_error(&q_id, &q_x).unwrap();
    assert!((err_x.angle_rad - angle).abs() < 1e-5);
    assert!((err_x.axis[0] - 1.0).abs() < 1e-5);
    assert!(err_x.axis[1].abs() < 1e-5);
    assert!(err_x.axis[2].abs() < 1e-5);
}
