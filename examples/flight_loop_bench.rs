//! Deterministic flight loop benchmark and telemetry serialization example.
//!
//! Exercises EKF prediction, attitude/GNSS correction updates, and
//! telemetry encoding/decoding over 50,000 deterministic iterations.

use std::time::Instant;

use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::{tilt_compensated_yaw, Mat};
use stm32_navcore::protocol::NavTelemetry;

/// Performance timing and update acceptance summary for flight loop benchmark.
#[derive(Debug, Clone, Copy)]
struct BenchMetrics {
    iterations: usize,
    total_micros: u128,
    p50_ns: u64,
    p99_ns: u64,
    mean_ns: u64,
    throughput_hz: f64,
    predict_accepted: usize,
    mag_accepted: usize,
    mag_rejected: usize,
    gnss_pos_accepted: usize,
    gnss_pos_rejected: usize,
    gnss_vel_accepted: usize,
    gnss_vel_rejected: usize,
}

fn synthetic_imu(step: usize) -> ([f32; 3], [f32; 3]) {
    let phase = (step as f32) * 0.001;
    let accel = [libm::sinf(phase) * 0.1, libm::cosf(phase) * 0.1, -9.80665];
    let gyro = [0.005 * libm::cosf(phase), -0.005 * libm::sinf(phase), 0.01];
    (accel, gyro)
}

fn synthetic_mag(step: usize) -> [f32; 3] {
    let heading = (step as f32) * 0.001 * 0.01;
    [libm::cosf(heading) * 25.0, libm::sinf(heading) * 25.0, 42.0]
}

fn synthetic_gnss(step: usize) -> ([f32; 3], [f32; 3]) {
    let t = (step as f32) * 0.001;
    let pos = [t * 0.5, t * 0.1, -10.0];
    let vel = [0.5, 0.1, 0.0];
    (pos, vel)
}

