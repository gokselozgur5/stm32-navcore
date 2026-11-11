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
