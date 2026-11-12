use stm32_navcore::fdir::{DualImuManager, ImuId, ImuReading};

const GOOD: ImuReading = ImuReading {
    accel_mps2: [0.0, 0.0, -9.81],
    gyro_rps: [0.0; 3],
};

#[test]
fn nan_gyro_dropout_fails_over_after_limit() {
    let mut mgr = DualImuManager::default_config();
    let mut dead = GOOD;
    dead.gyro_rps[1] = f32::NAN;
    for _ in 0..3 {
        mgr.update_and_select(&dead, &GOOD, [0.01; 3]);
    }
    assert_eq!(mgr.active_imu, ImuId::Secondary);
    assert!(!mgr.primary_healthy);
}
