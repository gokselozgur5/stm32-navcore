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

