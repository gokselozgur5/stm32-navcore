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
