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
