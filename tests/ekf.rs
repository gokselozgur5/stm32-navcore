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
fn test_ekf_zupt_damps_velocity_drift_under_attitude_tilt() {
    let mut ekf = NavEkf::new();
    let roll_offset = 0.02f32;
    ekf.att = stm32_navcore::math::Quaternion::new(
        libm::cosf(0.5 * roll_offset),
        libm::sinf(0.5 * roll_offset),
        0.0,
        0.0,
    );
    let dt = 0.01;
    let accel = [0.0, 0.0, -9.80665];
    let gyro = [0.0, 0.0, 0.0];
    let mut detector = stm32_navcore::ekf::StationaryDetector::new(10, 0.05, 0.03);
    for _ in 0..5000 {
        ekf.predict(accel, gyro, dt).unwrap();
        if detector.update(accel, gyro) {
            let _ = ekf.update_zupt(0.0001, 25.0);
        }
    }
    assert!(
        ekf.v_ned_mps[1].abs() < 1e-4,
        "v_ned_mps[1] {} exceeds 1e-4 m/s threshold under 0.02 rad roll tilt",
        ekf.v_ned_mps[1]
    );
}
