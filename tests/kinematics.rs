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

#[test]
fn test_gimbal_lock_singularity_detection() {
    let pitch_near_90 = core::f32::consts::FRAC_PI_2;
    let locked = EulerAngles::new(0.0, pitch_near_90, 0.0);
    assert!(locked.is_gimbal_locked());

    let res = body_rates_to_euler_rates(locked, [0.1, 0.0, 0.0]);
    match res {
        Err(KinematicsError::GimbalLock { pitch_rad }) => {
            assert!((pitch_rad - pitch_near_90).abs() < 1e-6);
        }
        _ => panic!("Expected GimbalLock error"),
    }
}

#[test]
fn test_omega_matrix_and_quat_derivative() {
    let q = Quaternion::IDENTITY;
    let omega = [0.1, -0.2, 0.3];
    let q_dot = quat_derivative(&q, omega);

    assert_eq!(q_dot[0], 0.0);
    assert!((q_dot[1] - 0.05).abs() < 1e-6);
    assert!((q_dot[2] - (-0.1)).abs() < 1e-6);
    assert!((q_dot[3] - 0.15).abs() < 1e-6);

    let recovered_omega = quat_derivative_to_body_rates(&q, q_dot);
    assert!((recovered_omega[0] - omega[0]).abs() < 1e-6);
    assert!((recovered_omega[1] - omega[1]).abs() < 1e-6);
    assert!((recovered_omega[2] - omega[2]).abs() < 1e-6);
}

#[test]
fn test_transition_matrix_and_propagation() {
    let q0 = Quaternion::IDENTITY;
    let omega = [0.0, 0.0, core::f32::consts::PI];
    let dt = 0.5;
    let q1 = propagate_attitude_quaternion(&q0, omega, dt).unwrap();

    let euler = EulerAngles::from_quat(&q1);
    assert!(euler.roll.abs() < 1e-4);
    assert!(euler.pitch.abs() < 1e-4);
    assert!((euler.yaw - core::f32::consts::FRAC_PI_2).abs() < 1e-4);
}
