use stm32_navcore::attitude::{AttitudeError, Mahony};
use stm32_navcore::math::Quaternion;

#[test]
fn test_invalid_dt_and_non_finite_inputs_rejected() {
    let mut filter = Mahony::default();
    let g = [0.0, 0.0, 0.0];
    let a = [0.0, 0.0, -9.81];

    assert_eq!(filter.update(g, a, 0.0), Err(AttitudeError::InvalidInput));
    assert_eq!(filter.update(g, a, -0.01), Err(AttitudeError::InvalidInput));
    assert_eq!(
        filter.update(g, a, f32::NAN),
        Err(AttitudeError::InvalidInput)
    );
    assert_eq!(
        filter.update([f32::NAN, 0.0, 0.0], a, 0.01),
        Err(AttitudeError::NonFinite)
    );
    assert_eq!(
        filter.update(g, [0.0, f32::INFINITY, 0.0], 0.01),
        Err(AttitudeError::NonFinite)
    );
}

#[test]
fn test_tilted_initial_attitude_converges_below_half_degree() {
    let half_tilt = 30.0f32.to_radians() * 0.5;
    let q_init = Quaternion::new(libm::cosf(half_tilt), libm::sinf(half_tilt), 0.0, 0.0);
    let mut filter = Mahony::with_attitude(2.0, 0.0, q_init).unwrap();
    let static_accel = [0.0, 0.0, -9.81];
    let zero_gyro = [0.0, 0.0, 0.0];

    for _ in 0..1000 {
        filter.update(zero_gyro, static_accel, 0.01).unwrap();
    }

    let (roll, pitch, _) = filter.euler_angles_zyx();
    let max_rad = 0.5f32.to_radians();
    assert!(roll.abs() < max_rad, "roll: {}", roll.to_degrees());
    assert!(pitch.abs() < max_rad, "pitch: {}", pitch.to_degrees());
}

#[test]
fn test_gyro_bias_converges_to_injected_offset() {
    let mut filter = Mahony::new(2.0, 0.1);
    let injected_bias = [0.02, -0.015, 0.0];
    let static_accel = [0.0, 0.0, -9.81];

    for _ in 0..6000 {
        filter.update(injected_bias, static_accel, 0.01).unwrap();
    }

    let est_bias = filter.gyro_bias();
    assert!((est_bias[0] - injected_bias[0]).abs() < 0.003);
    assert!((est_bias[1] - injected_bias[1]).abs() < 0.003);
}

#[test]
fn test_accessors_and_bias_reset() {
    let mut filter = Mahony::default();
    assert_eq!(filter.quaternion(), Quaternion::IDENTITY);
    filter.set_gyro_bias([0.01, -0.02, 0.03]);
    assert_eq!(filter.gyro_bias(), [0.01, -0.02, 0.03]);
    filter.reset_gyro_bias();
    assert_eq!(filter.gyro_bias(), [0.0; 3]);

    let (r, p, y) = filter.euler_angles_zyx();
    assert_eq!((r, p, y), (0.0, 0.0, 0.0));
}
