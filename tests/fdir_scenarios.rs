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
