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
