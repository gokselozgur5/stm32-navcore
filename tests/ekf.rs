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
