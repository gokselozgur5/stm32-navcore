use stm32_navcore::attitude::slerp::attitude_slerp;
use stm32_navcore::math::Quaternion;

#[test]
fn test_attitude_slerp_endpoints() {
    let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    assert_eq!(attitude_slerp(&q0, &q1, 0.0).unwrap(), q0);
}

#[test]
fn test_attitude_slerp_endpoints_full() {
    let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    assert_eq!(attitude_slerp(&q0, &q1, 1.0).unwrap(), q1);
}

#[test]
fn test_attitude_slerp_shortest_path() {
    let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q1 = Quaternion::new(-1.0, 0.0, 0.0, 0.0);
    let interpolated = attitude_slerp(&q0, &q1, 0.5).unwrap();
    assert!((interpolated.w.abs() - 1.0).abs() < 1e-5);
}

#[test]
fn test_attitude_slerp_collinear_fallback() {
    let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q1 = Quaternion::new(1.0, 1e-6, 0.0, 0.0).normalize().unwrap();
    let res = attitude_slerp(&q0, &q1, 0.5);
    assert!(res.is_ok());
}

#[test]
fn test_attitude_slerp_invalid_inputs() {
    let q_valid = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q_bad_norm = Quaternion::new(2.0, 0.0, 0.0, 0.0);
    assert!(attitude_slerp(&q_valid, &q_bad_norm, 0.5).is_err());
    assert!(attitude_slerp(&q_valid, &q_valid, -0.1).is_err());
    assert!(attitude_slerp(&q_valid, &q_valid, 1.1).is_err());
    assert!(attitude_slerp(&q_valid, &q_valid, f32::NAN).is_err());
}
