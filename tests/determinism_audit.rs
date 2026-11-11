use core::mem::size_of;
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::Quaternion;
use stm32_navcore::protocol::{CanFrame20B, NavTelemetry};

const TEST_STEPS: usize = 1000;
const DT: f32 = 0.01;
const GOLDEN_FNV1A_DIGEST: u64 = 0xb363_8a42_504f_301a;

#[test]
fn test_memory_budget_and_alignment_invariants() {
    assert!(size_of::<NavEkf>() <= 512, "NavEkf exceeds 512B budget");
    assert!(
        size_of::<CanFrame20B>() <= 16,
        "CanFrame20B exceeds 16B budget"
    );
    assert_eq!(core::mem::align_of::<NavEkf>(), 4);
    assert_eq!(core::mem::align_of::<Quaternion>(), 4);
}

fn synthetic_sensor_sample(step: usize) -> ([f32; 3], [f32; 3], f32, [f32; 3]) {
    let t = (step as f32) * DT;
    let accel = [
        0.005 * libm::sinf(t * 0.5),
        0.002 * libm::cosf(t * 0.3),
        -9.80665 + 0.001 * libm::sinf(t * 0.1),
    ];
    let gyro = [
        0.001 * libm::cosf(t * 0.2),
        -0.001 * libm::sinf(t * 0.4),
        0.0005 * libm::cosf(t * 0.1),
    ];
    let yaw = 0.05 * libm::sinf(t * 0.1);
    let gnss = [
        0.01 * libm::sinf(t * 0.2),
        0.01 * libm::cosf(t * 0.2),
        -0.005 * t,
    ];
    (accel, gyro, yaw, gnss)
}

fn fnv1a_feed(mut hash: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        hash ^= b as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

#[test]
fn test_bit_exact_ekf_simulation_determinism() {
    let mut ekf_a = NavEkf::new();
    let mut ekf_b = NavEkf::new();

    for step in 0..TEST_STEPS {
        let (accel, gyro, yaw, gnss) = synthetic_sensor_sample(step);
        ekf_a.predict(accel, gyro, DT).expect("predict failed");
        ekf_b.predict(accel, gyro, DT).expect("predict failed");

        if step % 5 == 0 {
            ekf_a.update_mag_yaw(yaw, 0.02, 16.0).expect("mag");
            ekf_b.update_mag_yaw(yaw, 0.02, 16.0).expect("mag");
        }
        if step % 10 == 0 {
            ekf_a.update_gnss_pos(gnss, 2.5, 16.0).expect("gnss");
            ekf_b.update_gnss_pos(gnss, 2.5, 16.0).expect("gnss");
        }
