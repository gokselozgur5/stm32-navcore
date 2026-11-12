//! Strapdown EKF predict and GNSS update loop.
//!
//! Propagates `NavEkf` with simulated stationary IMU samples at 100 Hz and
//! applies a GNSS position fix every 20 samples (5 Hz).
//!
//! Run with `cargo run --example ekf_predict_update`.

use stm32_navcore::ekf::NavEkf;

/// IMU sample rate in Hz.
const IMU_RATE_HZ: f32 = 100.0;
/// One GNSS fix per this many IMU samples.
const GNSS_EVERY_N_IMU: usize = 20;
/// Simulated duration in seconds.
const DURATION_S: f32 = 10.0;
/// Specific force of a level, stationary vehicle in body axes (m/s^2).
const STATIC_ACCEL_MPS2: [f32; 3] = [0.0, 0.0, -9.80665];
/// Gyro output of a stationary vehicle (rad/s).
const STATIC_GYRO_RADS: [f32; 3] = [0.0, 0.0, 0.0];
/// GNSS position variance per axis (m^2).
const GNSS_POS_VAR_M2: f32 = 1.0;
/// Chi-square gate applied to each scalar position update.
const NIS_GATE: f32 = 16.0;
/// Acceptance bound for the final position error in metres.
const MAX_FINAL_ERROR_M: f32 = 0.5;

/// Returns a deterministic GNSS position fix for fix index `k`.
///
/// The alternating +/-0.3 m offset stands in for measurement noise, so the
/// example is reproducible without a random number generator.
fn gnss_fix(k: usize) -> [f32; 3] {
    let offset = if k.is_multiple_of(2) { 0.3 } else { -0.3 };
    [offset, -offset, 0.0]
}
