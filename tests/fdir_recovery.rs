use stm32_navcore::fdir::{DualImuManager, ImuId, ImuReading};

const GOOD: ImuReading = ImuReading {
    accel_mps2: [0.0, 0.0, -9.81],
    gyro_rps: [0.0; 3],
};
const BAD: ImuReading = ImuReading {
    accel_mps2: [0.0, 0.0, -9.81],
    gyro_rps: [2.5, -3.0, 1.8],
};

fn failed_over(recovery: u32) -> DualImuManager {
    let mut m = DualImuManager::default_config().with_recovery(recovery);
    for _ in 0..3 {
        m.update_and_select(&BAD, &GOOD, [0.01; 3]);
    }
    assert_eq!(m.active_imu, ImuId::Secondary);
    m
}

#[test]
fn primary_recovers_after_required_clean_epochs() {
    let mut m = failed_over(4);
    for _ in 0..3 {
        m.update_and_select(&GOOD, &GOOD, [0.01; 3]);
        assert_eq!(m.active_imu, ImuId::Secondary);
    }
    m.update_and_select(&GOOD, &GOOD, [0.01; 3]);
    assert_eq!(m.active_imu, ImuId::Primary);
    assert!(m.primary_healthy);
}

#[test]
fn fault_during_recovery_restarts_the_window() {
    let mut m = failed_over(3);
    m.update_and_select(&GOOD, &GOOD, [0.01; 3]);
    m.update_and_select(&BAD, &GOOD, [0.01; 3]);
    assert_eq!(m.clean_epochs, 0);
    m.update_and_select(&GOOD, &GOOD, [0.01; 3]);
    assert_eq!(m.active_imu, ImuId::Secondary);
}

