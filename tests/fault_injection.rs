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
