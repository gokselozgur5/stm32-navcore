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
