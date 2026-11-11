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

/// Execute closed-loop navigation iterations and record step latencies.
fn run_benchmark(iterations: usize) -> (BenchMetrics, u64) {
    let mut ekf = NavEkf::new();
    let mut latencies_ns = Vec::with_capacity(iterations);
    let mut dummy_checksum: u64 = 0;
    let mut uart_buf = [0u8; 64];
    let dt = 0.001;
    let hard_iron = [0.0f32; 3];
    let soft_iron = Mat::identity();

    let mut predict_accepted = 0;
    let mut mag_accepted = 0;
    let mut mag_rejected = 0;
    let mut gnss_pos_accepted = 0;
    let mut gnss_pos_rejected = 0;
    let mut gnss_vel_accepted = 0;
    let mut gnss_vel_rejected = 0;

    let t_start = Instant::now();
    for step in 0..iterations {
        let step_start = Instant::now();
        let (accel, gyro) = synthetic_imu(step);
        if ekf.predict(accel, gyro, dt).is_ok() {
            predict_accepted += 1;
        }

        if step % 10 == 0 {
            let mag = synthetic_mag(step);
            let (roll, pitch, _) = ekf.att.to_euler_zyx();
            if let Ok(yaw_meas) = tilt_compensated_yaw(mag, hard_iron, soft_iron, roll, pitch) {
                match ekf.update_mag_yaw(yaw_meas, 0.05, 9.0) {
                    Ok(_) => mag_accepted += 1,
                    Err(_) => mag_rejected += 1,
                }
            }
        }

        if step % 50 == 0 {
            let (pos_m, vel_m) = synthetic_gnss(step);
            match ekf.update_gnss_pos(pos_m, 1.0, 16.0) {
                Ok(_) => gnss_pos_accepted += 1,
                Err(_) => gnss_pos_rejected += 1,
            }
            match ekf.update_gnss_vel(vel_m, 0.1, 16.0) {
                Ok(_) => gnss_vel_accepted += 1,
                Err(_) => gnss_vel_rejected += 1,
            }
        }
