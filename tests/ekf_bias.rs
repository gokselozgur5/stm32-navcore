//! Gyro bias observability and convergence tests for the navigation EKF.

use stm32_navcore::ekf::NavEkf;

const DT: f32 = 0.02;
const GRAVITY_BODY: [f32; 3] = [0.0, 0.0, -9.80665];

fn run_with_heading_aiding(true_bias: f32, epochs: u32) -> NavEkf {
    let mut ekf = NavEkf::new();
    for _ in 0..epochs {
        ekf.predict(GRAVITY_BODY, [0.0, 0.0, true_bias], DT)
            .unwrap();
        let _ = ekf.update_mag_yaw(0.0, 0.001, 100.0);
    }
    ekf
}

#[test]
fn gyro_bias_is_unobservable_without_heading_aiding() {
    let mut ekf = NavEkf::new();
    let initial = ekf.gyro_bias_sigma();
    for step in 0..600u32 {
        ekf.predict(GRAVITY_BODY, [0.0, 0.0, 0.02], DT).unwrap();
        if step % 5 == 0 {
            ekf.update_gnss_pos([0.0; 3], 1.0, 1e6).unwrap();
            ekf.update_gnss_vel([0.0; 3], 0.1, 1e6).unwrap();
        }
        assert!(ekf.gyro_bias_sigma() >= initial, "step {}", step);
    }
}
