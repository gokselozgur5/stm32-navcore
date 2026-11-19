use stm32_navcore::fdir::{DualImuManager, ImuId, ImuReading};

#[test]
fn finite_accel_fault_switches_active_imu() {
    let mut mgr = DualImuManager::default_config();
    let primary = ImuReading {
        accel_mps2: [0.0, 0.0, 40.0],
        gyro_rps: [0.0; 3],
    };
    let secondary = ImuReading {
        accel_mps2: [0.0, 0.0, -9.81],
        gyro_rps: [0.0; 3],
    };
    for _ in 0..10 {
        mgr.update_and_select(&primary, &secondary, [0.01; 3]);
    }
    assert_eq!(mgr.active_imu, ImuId::Secondary);
}
