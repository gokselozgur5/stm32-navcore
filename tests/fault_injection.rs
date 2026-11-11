//! Adversarial Sensor Fault-Injection & Joseph Covariance Stress Suite.

use stm32_navcore::ekf::{EkfError, NavEkf, STATE_DIM};
use stm32_navcore::protocol::{cobs_decode, NavTelemetry, ProtocolError};

#[test]
fn test_adversarial_imu_burst_rejection() {
    let mut ekf = NavEkf::new();
    let before = ekf;
    assert_eq!(
        ekf.predict([f32::NAN, 0.0, -9.80665], [0.0; 3], 0.01),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    assert_eq!(
        ekf.predict([f32::INFINITY, 0.0, -9.80665], [0.0; 3], 0.01),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    assert_eq!(
        ekf.predict([0.0, 0.0, -9.80665], [0.0, f32::NAN, 0.0], 0.01),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    let bad_dt = [0.0, -0.01, f32::NAN, f32::INFINITY];
    for &dt in &bad_dt {
        assert_eq!(
            ekf.predict([0.0, 0.0, -9.80665], [0.0; 3], dt),
            Err(EkfError::InvalidInput)
        );
        assert_eq!(ekf, before);
    }
}

#[test]
fn test_adversarial_gnss_pos_burst_rejection() {
    let mut ekf = NavEkf::new();
    let before = ekf;
    assert_eq!(
        ekf.update_gnss_pos([f32::NAN, 0.0, 0.0], 1.0, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    assert_eq!(
        ekf.update_gnss_pos([0.0, f32::INFINITY, 0.0], 1.0, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
}

#[test]
fn test_adversarial_gnss_pos_variance_rejection() {
    let mut ekf = NavEkf::new();
    let before = ekf;
    assert_eq!(
        ekf.update_gnss_pos([1.0, 1.0, 1.0], -1.0, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    assert_eq!(
        ekf.update_gnss_pos([1.0, 1.0, 1.0], 1.0, -16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
}

#[test]
fn test_adversarial_gnss_vel_burst_rejection() {
    let mut ekf = NavEkf::new();
    let before = ekf;
    assert_eq!(
        ekf.update_gnss_vel([0.0, 0.0, f32::NAN], 0.5, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    assert_eq!(
        ekf.update_gnss_vel([0.0, 0.0, f32::NEG_INFINITY], 0.5, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
}

#[test]
fn test_adversarial_mag_yaw_rejection() {
    let mut ekf = NavEkf::new();
    let before = ekf;
    assert_eq!(
        ekf.update_mag_yaw(f32::NAN, 0.05, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
    assert_eq!(
        ekf.update_mag_yaw(f32::INFINITY, 0.05, 16.0),
        Err(EkfError::InvalidInput)
    );
    assert_eq!(ekf, before);
}

#[test]
fn test_high_magnitude_finite_imu_predict() {
    let mut ekf = NavEkf::new();
    let res = ekf.predict([3.0e38, 0.0, -9.80665], [0.0; 3], 0.01);
    assert!(res.is_ok());
    assert!(ekf.p_ned_m[0].is_finite());
    assert!(ekf.p_ned_m[0] > 1.0e34);
}

#[allow(clippy::manual_is_multiple_of)]
fn step_stress_ekf(ekf: &mut NavEkf, s: usize, dt: f32) -> bool {
    let t = s as f32 * dt;
    let ax = 5.0 * libm::sinf(2.0 * t);
    let az = -9.80665 + libm::sinf(t);
    ekf.predict([ax, 0.0, az], [0.0, 0.0, 0.5], dt).unwrap();
    if s % 20 == 0 {
        let obs = if s % 200 == 0 {
            [500.0, 0.0, 0.0]
        } else {
            [ax * 0.1, 0.0, 0.0]
        };
        ekf.update_gnss_pos(obs, 1.0, 25.0).is_ok()
    } else {
        true
    }
}

#[test]
fn test_5000_step_joseph_covariance_stress() {
    let mut ekf = NavEkf::new();
    let dt = 0.01;
    let (mut acc, mut rej) = (0, 0);
    for s in 0..5000 {
        if step_stress_ekf(&mut ekf, s, dt) {
            acc += 1;
        } else {
            rej += 1;
        }
        for i in 0..STATE_DIM {
            assert!(ekf.cov.0[i][i] > 0.0 && ekf.cov.0[i][i].is_finite());
            for j in 0..STATE_DIM {
                assert_eq!(ekf.cov.0[i][j], ekf.cov.0[j][i]);
            }
        }
    }
    assert!(acc > 0 && rej > 0);
}

#[test]
fn test_wire_fuzz_cobs_decoding() {
    let mut dst = [0u8; 128];
    assert_eq!(
        cobs_decode(&[], &mut dst),
        Err(ProtocolError::InvalidCobsEncoding)
    );
    let inv = [0x05, 0x01, 0x02];
    assert_eq!(
        cobs_decode(&inv, &mut dst),
        Err(ProtocolError::InvalidCobsEncoding)
    );
    let mut seed = 0x12345678u32;
    for _ in 0..2000 {
        let mut buf = [0u8; 32];
        for b in &mut buf {
            seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
            *b = (seed >> 16) as u8;
        }
        let _ = cobs_decode(&buf, &mut dst);
        let _ = NavTelemetry::unpack_uart(&buf);
    }
}

#[test]
fn test_exhaustive_bit_flip_rejection_uart() {
    let telemetry = NavTelemetry {
        timestamp_ms: 123456,
        pos_ned_m: [10.5, -20.2, -5.0],
        vel_ned_mps: [1.2, -0.5, 0.1],
        euler_rad: [0.05, -0.02, 1.57],
        status: 0x03,
    };
    let mut uart_buf = [0u8; 64];
    let len = telemetry.pack_uart(&mut uart_buf).unwrap();
    for byte_idx in 0..len {
        for bit_idx in 0..8 {
            let mut corrupted = [0u8; 64];
            corrupted[..len].copy_from_slice(&uart_buf[..len]);
            corrupted[byte_idx] ^= 1 << bit_idx;
            assert!(NavTelemetry::unpack_uart(&corrupted[..len]).is_err());
        }
    }
}

#[test]
fn test_exhaustive_bit_flip_rejection_can() {
    let telemetry = NavTelemetry {
        timestamp_ms: 123456,
        pos_ned_m: [10.5, -20.2, -5.0],
        vel_ned_mps: [1.2, -0.5, 0.1],
        euler_rad: [0.05, -0.02, 1.57],
        status: 0x03,
    };
    let frames = telemetry.pack_can().unwrap();
    for frame_idx in 0..6 {
        for byte_idx in 0..frames[frame_idx].dlc as usize {
            for bit_idx in 0..8 {
                let mut corrupted = frames;
                corrupted[frame_idx].data[byte_idx] ^= 1 << bit_idx;
                assert!(NavTelemetry::unpack_can(&corrupted).is_err());
            }
        }
    }
}
