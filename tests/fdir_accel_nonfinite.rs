use stm32_navcore::ekf::NavEkf;
use stm32_navcore::fdir::{DualImuManager, ImuId, ImuReading};

const GOOD: ImuReading = ImuReading {
    accel_mps2: [0.0, 0.0, -9.81],
    gyro_rps: [0.0; 3],
};

fn run_dead_accel(value: f32) -> (DualImuManager, NavEkf) {
    let mut mgr = DualImuManager::default_config();
    let mut ekf = NavEkf::new();
    let mut dead = GOOD;
    dead.accel_mps2[2] = value;
    for _ in 0..10 {
        let r = mgr.update_and_select(&dead, &GOOD, [0.01; 3]);
        let _ = ekf.predict(r.accel_mps2, r.gyro_rps, 0.01);
    }
    (mgr, ekf)
}

#[test]
fn nan_primary_accel_fails_over() {
    let (mgr, mut ekf) = run_dead_accel(f32::NAN);
    assert_eq!(mgr.active_imu, ImuId::Secondary);
    assert!(ekf.predict(GOOD.accel_mps2, GOOD.gyro_rps, 0.01).is_ok());
}
