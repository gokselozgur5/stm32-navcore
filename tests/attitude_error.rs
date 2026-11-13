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

#[test]
fn test_attitude_error_sign_invariance() {
    let angle = 0.75f32;
    let half = angle * 0.5;
    let q = Quaternion::new(libm::cosf(half), 0.0, libm::sinf(half), 0.0);
    let q_neg = Quaternion::new(-q.w, -q.x, -q.y, -q.z);

    let err_pos = attitude_error(&Quaternion::IDENTITY, &q).unwrap();
    let err_neg = attitude_error(&Quaternion::IDENTITY, &q_neg).unwrap();

    assert!((err_pos.angle_rad - err_neg.angle_rad).abs() < 1e-6);
    assert!((err_pos.angle_rad - angle).abs() < 1e-5);
    assert!((err_pos.axis[1] - 1.0).abs() < 1e-5);
    assert!((err_neg.axis[1] - 1.0).abs() < 1e-5);
}

#[test]
fn test_attitude_error_oblique_axis_and_invalid() {
    let axis = [1.0 / libm::sqrtf(3.0); 3];
    let angle = 1.25f32;
    let half = angle * 0.5;
    let sin_h = libm::sinf(half);
    let q = Quaternion::new(
        libm::cosf(half),
        axis[0] * sin_h,
        axis[1] * sin_h,
        axis[2] * sin_h,
    );

    let err = attitude_error(&Quaternion::IDENTITY, &q).unwrap();
    assert!((err.angle_rad - angle).abs() < 1e-5);
    for i in 0..3 {
        assert!((err.axis[i] - axis[i]).abs() < 1e-5);
    }

    let nan_q = Quaternion::new(f32::NAN, 0.0, 0.0, 0.0);
    assert!(attitude_error(&nan_q, &q).is_err());
    assert!(attitude_error(&q, &nan_q).is_err());
}
