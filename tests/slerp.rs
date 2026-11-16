use stm32_navcore::math::Quaternion;
use stm32_navcore::slerp::{quat_dot, quat_nlerp};

#[test]
fn test_nlerp_endpoints_and_midpoint() {
    let q0 = Quaternion::IDENTITY;
    let q1 = Quaternion::new(0.0, 0.0, 1.0, 0.0);
    let start = quat_nlerp(&q0, &q1, 0.0).unwrap();
    assert!((quat_dot(&start, &q0) - 1.0).abs() < 1e-6);
    let end = quat_nlerp(&q0, &q1, 1.0).unwrap();
    assert!((quat_dot(&end, &q1) - 1.0).abs() < 1e-6);
    let mid = quat_nlerp(&q0, &q1, 0.5).unwrap();
    let norm = (mid.w * mid.w + mid.x * mid.x + mid.y * mid.y + mid.z * mid.z).sqrt();
    assert!((norm - 1.0).abs() < 1e-6);
}

#[test]
fn test_slerp_endpoints_and_midpoint() {
    let q0 = Quaternion::IDENTITY;
    let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let start = stm32_navcore::slerp::quat_slerp(&q0, &q1, 0.0).unwrap();
    assert!((quat_dot(&start, &q0) - 1.0).abs() < 1e-6);
    let end = stm32_navcore::slerp::quat_slerp(&q0, &q1, 1.0).unwrap();
    assert!((quat_dot(&end, &q1) - 1.0).abs() < 1e-6);
    let mid = stm32_navcore::slerp::quat_slerp(&q0, &q1, 0.5).unwrap();
    let expected = core::f32::consts::FRAC_1_SQRT_2;
    assert!((mid.w - expected).abs() < 1e-5);
    assert!((mid.x - expected).abs() < 1e-5);
}

#[test]
fn test_slerp_shortest_path_antipodal() {
    let q0 = Quaternion::IDENTITY;
    let q1 = Quaternion::new(-1.0, 0.0, 0.0, 0.0);
    let mid = stm32_navcore::slerp::quat_slerp(&q0, &q1, 0.5).unwrap();
    assert!((mid.w.abs() - 1.0).abs() < 1e-5);
    assert!(mid.x.abs() < 1e-5);
    assert!(mid.y.abs() < 1e-5);
    assert!(mid.z.abs() < 1e-5);
}

#[test]
fn test_slerp_rejections_and_invalid_inputs() {
    let q_valid = Quaternion::IDENTITY;
    let q_zero = Quaternion::new(0.0, 0.0, 0.0, 0.0);
    let q_nan = Quaternion::new(f32::NAN, 0.0, 0.0, 0.0);

    assert!(stm32_navcore::slerp::quat_slerp(&q_zero, &q_valid, 0.5).is_err());
    assert!(stm32_navcore::slerp::quat_slerp(&q_valid, &q_nan, 0.5).is_err());
    assert!(stm32_navcore::slerp::quat_slerp(&q_valid, &q_valid, f32::NAN).is_err());
    assert!(stm32_navcore::slerp::quat_nlerp(&q_zero, &q_valid, 0.5).is_err());
}

#[test]
fn test_slerp_near_collinear_fallback() {
    let q0 = Quaternion::IDENTITY;
    let angle = 0.0001_f32;
    let half_a = angle * 0.5;
    let q1 = Quaternion::new(libm::cosf(half_a), libm::sinf(half_a), 0.0, 0.0);
    let interpolated = stm32_navcore::slerp::quat_slerp(&q0, &q1, 0.5).unwrap();
    let norm = (interpolated.w * interpolated.w + interpolated.x * interpolated.x).sqrt();
    assert!((norm - 1.0).abs() < 1e-6);
    assert!((interpolated.x - half_a * 0.5).abs() < 1e-6);
}
