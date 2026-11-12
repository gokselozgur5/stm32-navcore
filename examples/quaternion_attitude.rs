//! Quaternion attitude propagation with a constant body rate.
//!
//! Integrates a constant yaw rate at 100 Hz using `Quaternion::integrate_gyro`
//! and checks the resulting yaw against the analytic value.
//!
//! Run with `cargo run --example quaternion_attitude`.

use stm32_navcore::math::Quaternion;

/// IMU gyro sample rate in Hz.
const IMU_RATE_HZ: f32 = 100.0;
/// Simulated duration in seconds.
const DURATION_S: f32 = 4.0;
/// Constant body rate about the Z axis in rad/s (about 28.6 deg/s).
const YAW_RATE_RAD_S: f32 = 0.5;
/// Acceptance tolerance for the final yaw in rad.
const TOLERANCE_RAD: f32 = 1e-3;
