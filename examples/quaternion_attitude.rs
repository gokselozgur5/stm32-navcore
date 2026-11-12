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

/// Advances the attitude by one gyro sample.
///
/// The integrator rejects non-finite or negative time steps. In that case
/// the previous attitude is kept, so the state never becomes invalid.
fn step(q: Quaternion, gyro_rad_s: [f32; 3], dt_s: f32) -> Quaternion {
    q.integrate_gyro(gyro_rad_s, dt_s).unwrap_or(q)
}

fn main() {
    let dt_s = 1.0 / IMU_RATE_HZ;
    let steps = (DURATION_S * IMU_RATE_HZ) as usize;
    let gyro = [0.0, 0.0, YAW_RATE_RAD_S];

    // Start from the identity attitude and integrate the constant rate.
    let mut q = Quaternion::IDENTITY;
    for _ in 0..steps {
        q = step(q, gyro, dt_s);
    }

    let (roll, pitch, yaw) = q.to_euler_zyx();
    let expected_yaw = YAW_RATE_RAD_S * DURATION_S;
    println!("norm after {steps} steps: {:.6}", q.norm());
    println!("roll  = {roll:.6} rad");
    println!("pitch = {pitch:.6} rad");
    println!("yaw   = {yaw:.6} rad (expected {expected_yaw:.6})");

    assert!((yaw - expected_yaw).abs() < TOLERANCE_RAD, "yaw drifted");
}
