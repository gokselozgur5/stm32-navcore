use stm32_navcore::ekf::NavEkf;

#[test]
fn test_ekf_roll_pitch_bounded_under_gyro_bias() {
    let mut ekf = NavEkf::new();
    for step in 0..6000 {
        ekf.predict([0.0, 0.0, -9.80665], [0.002, 0.0, 0.0], 0.01)
            .unwrap();
        if step % 20 == 0 {
            let _ = ekf.update_gnss_pos([0.0, 0.0, 0.0], 1.0, 16.0);
            let _ = ekf.update_gnss_vel([0.0, 0.0, 0.0], 0.1, 16.0);
        }
        if step % 10 == 0 {
            let _ = ekf.update_mag_yaw(0.0, 0.01, 16.0);
        }
    }
    let (roll, _pitch, _yaw) = ekf.att.to_euler_zyx();
    assert!(
        roll.abs() < 0.01,
        "roll {} exceeds 0.01 rad bound under 0.002 rad/s gyro bias",
        roll
    );
}

#[test]
fn test_quat_to_euler_zyx_exact_gimbal_lock_heading() {
    let q = stm32_navcore::math::Quaternion::new(-0.5, 0.5, 0.5, 0.5);
    let (roll, pitch, yaw) = q.to_euler_zyx();
    assert!((roll - 0.0).abs() < 1e-5);
    assert!((pitch - (-core::f32::consts::FRAC_PI_2)).abs() < 1e-5);
    assert!((yaw - (-core::f32::consts::FRAC_PI_2)).abs() < 1e-5);
}

#[test]
fn test_dual_antenna_heading_custom_gate_rejection() {
    let mut ekf = NavEkf::new();
    let res = ekf.update_dual_antenna_heading(1.0, 0.01, 1.0);
    assert!(res.is_err());
}

#[test]
fn test_ekf_stationary_equator_gravity_drift_repro() {
    let mut ekf = NavEkf::new();
    let g_eq = 9.780325f32; // Somigliana at equator
    let dt = 0.01;
    for _ in 0..6000 {
        ekf.predict_with_gravity([0.0, 0.0, -g_eq], [0.0, 0.0, 0.0], dt, g_eq)
            .unwrap();
    }
    // A stationary unit at the equator measures specific force -9.7803 m/s^2.
    // If fixed 9.80665 is assumed, predict integrates spurious +0.0263 m/s^2,
    // causing ~47 m drift after 60 s (0.5 * 0.0263 * 60^2).
    // The vertical position error must be within 1.0 m under true local gravity.
    assert!(
        ekf.p_ned_m[2].abs() < 1.0,
        "down position drifted excessively due to fixed gravity: {}",
        ekf.p_ned_m[2]
    );
}

#[test]
fn test_accel_bias_estimator_equator_gravity_repro() {
    use stm32_navcore::ekf::AccelBiasEstimator;
    use stm32_navcore::math::Quaternion;
    let mut est = AccelBiasEstimator::new(1.0, 0.01);
    let g_eq = 9.780325f32;
    let accel = [0.0, 0.0, -g_eq];
    for _ in 0..500 {
        est.update_with_gravity(&Quaternion::IDENTITY, accel, g_eq)
            .unwrap();
    }
    // AccelBiasEstimator learns a false ~+0.026 m/s^2 z bias at the equator
    // when using standard fixed gravity instead of equator normal gravity.
    assert!(
        est.bias_mps2[2].abs() < 0.005,
        "learned false z-bias at equator: {}",
        est.bias_mps2[2]
    );
}
