use stm32_navcore::kinematics::*;
use stm32_navcore::math::Quaternion;

#[test]
fn test_euler_quat_roundtrip() {
    let angles = EulerAngles::new(0.15, -0.25, 1.20);
    let q = angles.to_quat();
    let norm = libm::sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    assert!((norm - 1.0).abs() < 1e-6);

    let recovered = EulerAngles::from_quat(&q);
    assert!((recovered.roll - angles.roll).abs() < 1e-5);
    assert!((recovered.pitch - angles.pitch).abs() < 1e-5);
    assert!((recovered.yaw - angles.yaw).abs() < 1e-5);
}

#[test]
fn test_identity_quaternion_euler() {
    let q = Quaternion::IDENTITY;
    let angles = EulerAngles::from_quat(&q);
    assert_eq!(angles.roll, 0.0);
    assert_eq!(angles.pitch, 0.0);
    assert_eq!(angles.yaw, 0.0);
}

#[test]
fn test_euler_rates_and_body_rates_inversion() {
    let euler = EulerAngles::new(0.2, -0.1, 0.5);
    let original_rates = [0.05, -0.02, 0.08];

    let body_rates = euler_rates_to_body_rates(euler, original_rates);
    let recovered_rates = body_rates_to_euler_rates(euler, body_rates).unwrap();

    assert!((recovered_rates[0] - original_rates[0]).abs() < 1e-5);
    assert!((recovered_rates[1] - original_rates[1]).abs() < 1e-5);
    assert!((recovered_rates[2] - original_rates[2]).abs() < 1e-5);
}
