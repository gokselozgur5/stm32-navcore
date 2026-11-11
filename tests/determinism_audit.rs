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
        for i in 0..3 {
            assert_eq!(ekf_a.p_ned_m[i].to_bits(), ekf_b.p_ned_m[i].to_bits());
            assert_eq!(ekf_a.v_ned_mps[i].to_bits(), ekf_b.v_ned_mps[i].to_bits());
        }
        assert_eq!(ekf_a.att.w.to_bits(), ekf_b.att.w.to_bits());
        assert_eq!(ekf_a.att.x.to_bits(), ekf_b.att.x.to_bits());
        assert_eq!(ekf_a.att.y.to_bits(), ekf_b.att.y.to_bits());
        assert_eq!(ekf_a.att.z.to_bits(), ekf_b.att.z.to_bits());
        assert_eq!(ekf_a.bg_z.to_bits(), ekf_b.bg_z.to_bits());

        for row in 0..8 {
            for col in 0..8 {
                assert_eq!(
                    ekf_a.cov.0[row][col].to_bits(),
                    ekf_b.cov.0[row][col].to_bits()
                );
            }
        }
    }
    let mut h: u64 = 0xcbf29ce484222325;
    for x in &ekf_a.p_ned_m {
        h = fnv1a_feed(h, &x.to_bits().to_le_bytes());
    }
    for x in &ekf_a.v_ned_mps {
        h = fnv1a_feed(h, &x.to_bits().to_le_bytes());
    }
    h = fnv1a_feed(h, &ekf_a.att.w.to_bits().to_le_bytes());
    h = fnv1a_feed(h, &ekf_a.att.x.to_bits().to_le_bytes());
    h = fnv1a_feed(h, &ekf_a.att.y.to_bits().to_le_bytes());
    h = fnv1a_feed(h, &ekf_a.att.z.to_bits().to_le_bytes());
    h = fnv1a_feed(h, &ekf_a.bg_z.to_bits().to_le_bytes());
    for row in 0..8 {
        for col in 0..8 {
            h = fnv1a_feed(h, &ekf_a.cov.0[row][col].to_bits().to_le_bytes());
        }
    }
    let (r, p, y) = ekf_a.att.to_euler_zyx();
    let telem = NavTelemetry {
        timestamp_ms: (TEST_STEPS * 10) as u32,
        pos_ned_m: ekf_a.p_ned_m,
        vel_ned_mps: ekf_a.v_ned_mps,
        euler_rad: [r, p, y],
        status: 1,
    };
    let mut uart_buf = [0u8; 128];
    let u_len = telem.pack_uart(&mut uart_buf).expect("pack uart");
    h = fnv1a_feed(h, &uart_buf[..u_len]);

    let can_frames = telem.pack_can().expect("pack can");
    for frame in &can_frames {
        h = fnv1a_feed(h, &frame.id.to_le_bytes());
        h = fnv1a_feed(h, &[frame.dlc]);
        h = fnv1a_feed(h, &frame.data);
    }
    assert_eq!(h, GOLDEN_FNV1A_DIGEST);
}

#[test]
fn test_bit_exact_protocol_encoding_determinism() {
    let mut ekf_a = NavEkf::new();
    let mut ekf_b = NavEkf::new();
    let mut uart_buf_a = [0u8; 128];
    let mut uart_buf_b = [0u8; 128];

    for step in 0..100 {
        let (accel, gyro, _, _) = synthetic_sensor_sample(step);
        ekf_a.predict(accel, gyro, DT).expect("predict failed");
        ekf_b.predict(accel, gyro, DT).expect("predict failed");

        let (r_a, p_a, y_a) = ekf_a.att.to_euler_zyx();
        let (r_b, p_b, y_b) = ekf_b.att.to_euler_zyx();
        let telem_a = NavTelemetry {
            timestamp_ms: (step * 10) as u32,
            pos_ned_m: ekf_a.p_ned_m,
            vel_ned_mps: ekf_a.v_ned_mps,
            euler_rad: [r_a, p_a, y_a],
            status: 1,
        };
