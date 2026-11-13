use stm32_navcore::attitude::Madgwick;
use stm32_navcore::math::Quaternion;

#[test]
fn test_madgwick_initialization() {
    let mut f = Madgwick::default();
    assert_eq!(f.quaternion(), Quaternion::IDENTITY);
    assert!((f.beta - Madgwick::DEFAULT_BETA).abs() < 1e-6);

    let custom = Madgwick::new(0.05);
    assert!((custom.beta - 0.05).abs() < 1e-6);

    let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    f.set_quaternion(q).unwrap();
    assert_eq!(f.quaternion(), q);
    f.reset();
    assert_eq!(f.quaternion(), Quaternion::IDENTITY);
}

#[test]
fn test_madgwick_invalid_inputs_and_gyro_only() {
    let mut f = Madgwick::new(0.1);
    assert!(f.update([0.0, 0.0, 0.0], [0.0, 0.0, 9.8], 0.0).is_err());
    assert!(f.update([0.0, 0.0, 0.0], [0.0, 0.0, 9.8], -0.01).is_err());
    assert!(f
        .update([f32::NAN, 0.0, 0.0], [0.0, 0.0, 9.8], 0.01)
        .is_err());

    let dt = 0.01;
    let rate = 0.1;
    for _ in 0..100 {
        f.update([rate, 0.0, 0.0], [0.0, 0.0, 0.0], dt).unwrap();
    }
    let (roll, pitch, yaw) = f.quaternion().to_euler_zyx();
    assert!((roll - 0.1).abs() < 0.02);
    assert!(pitch.abs() < 1e-3);
    assert!(yaw.abs() < 1e-3);
}

#[test]
fn test_madgwick_static_tilt_convergence() {
    let mut f = Madgwick::new(0.5);
    let target_pitch = 30.0f32.to_radians();
    let ax = -libm::sinf(target_pitch) * 9.80665;
    let az = libm::cosf(target_pitch) * 9.80665;
    let dt = 0.01;

    for _ in 0..1000 {
        f.update([0.0, 0.0, 0.0], [ax, 0.0, az], dt).unwrap();
    }
    let (roll, pitch, yaw) = f.quaternion().to_euler_zyx();
    assert!(roll.abs() < 0.5f32.to_radians());
    assert!((pitch - target_pitch).abs() < 0.5f32.to_radians());
    assert!(yaw.abs() < 0.5f32.to_radians());
}

#[test]
fn test_madgwick_norm_stability_100k_steps() {
    let mut f = Madgwick::default();
    let dt = 0.01;
    let gyro = [0.05, -0.03, 0.02];
    let accel = [0.2, -0.1, 9.78];

    for _ in 0..100_000 {
        let q = f.update(gyro, accel, dt).unwrap();
        let norm_err = (q.norm() - 1.0).abs();
        assert!(norm_err < 1e-5);
    }
}
