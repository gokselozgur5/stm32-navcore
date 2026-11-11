use core::f32::consts::PI;
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::{tilt_compensated_yaw, Mat, Quaternion};
use stm32_navcore::protocol::NavTelemetry;

struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0xDEAD_BEEF_CAFE_BABE
            } else {
                seed
            },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / 16777216.0
    }

    fn next_gaussian_approx(&mut self, std_dev: f32) -> f32 {
        let mut sum = 0.0f32;
        for _ in 0..12 {
            sum += self.next_f32();
        }
        (sum - 6.0) * std_dev
    }
}

fn wrap_pi(mut a: f32) -> f32 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

#[test]
fn test_closed_loop_hil_trajectory_and_gnss_outage() {
    let mut rng = XorShift64::new(42);
    let mut ekf = NavEkf::new();

    let total_steps = 6000;
    let dt = 0.01f32;
    let omega = 2.0 * PI / 60.0;
    let r_traj = 20.0f32;

    let hard_iron = [20.0f32, -15.0, 5.0];
    let soft_iron = Mat::<3, 3>([
        [1.05f32, 0.02, 0.0],
        [0.02, 0.98f32, 0.0],
        [0.0, 0.0, 1.0f32],
    ]);
    let inv_soft = soft_iron.inv_3x3().unwrap();
    let (bn, bd) = (20.0f32, 40.0f32);
    let true_gyro_bias_z = 0.005f32;

    let mut uart_buf = [0u8; NavTelemetry::MAX_UART_FRAME_LEN];
    let mut error_pos_xy_sum_sq = 0.0f32;
    let mut count_pos_xy = 0;

    let mut pre_outage_p_var = 0.0f32;
    let mut end_outage_p_var = 0.0f32;
    let mut max_pos_drift_outage = 0.0f32;
