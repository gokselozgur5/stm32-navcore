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
