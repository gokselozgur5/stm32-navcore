//! Exhaustive FDIR state machine transition scenarios.

use stm32_navcore::fdir::{DualImuManager, ImuId, ImuReading};

const VAR: [f32; 3] = [0.01; 3];

fn imu(gyro: [f32; 3]) -> ImuReading {
    ImuReading {
        accel_mps2: [0.0, 0.0, -9.81],
        gyro_rps: gyro,
    }
}

#[test]
fn spike_shorter_than_limit_is_ignored() {
    let mut m = DualImuManager::default_config();
    let spike = imu([5.0, 0.0, 0.0]);
    m.update_and_select(&spike, &imu([0.0; 3]), VAR);
    m.update_and_select(&spike, &imu([0.0; 3]), VAR);
    m.update_and_select(&imu([0.0; 3]), &imu([0.0; 3]), VAR);
    assert_eq!(m.active_imu, ImuId::Primary);
    assert_eq!(m.consecutive_faults, 0);
}

#[test]
fn intermittent_spikes_reset_the_counter() {
    let mut m = DualImuManager::default_config();
    let spike = imu([5.0, 0.0, 0.0]);
    for _ in 0..10 {
        m.update_and_select(&spike, &imu([0.0; 3]), VAR);
        m.update_and_select(&imu([0.0; 3]), &imu([0.0; 3]), VAR);
    }
    assert!(m.primary_healthy);
    assert_eq!(m.active_imu, ImuId::Primary);
}

#[test]
fn threshold_boundary_is_strictly_greater() {
    let mut m = DualImuManager::new(4.0, 3);
    let a = imu([0.0; 3]);
    assert!(!m.evaluate_sensors(&imu([2.0, 0.0, 0.0]), &a, [1.0; 3]));
    assert!(m.evaluate_sensors(&imu([2.01, 0.0, 0.0]), &a, [1.0; 3]));
}
