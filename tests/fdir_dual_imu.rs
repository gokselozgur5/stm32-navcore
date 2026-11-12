use stm32_navcore::fdir::{DualImuManager, ImuId, ImuReading};

const GOOD: ImuReading = ImuReading {
    accel_mps2: [0.0, 0.0, -9.81],
    gyro_rps: [0.0; 3],
};
const NAN_GYRO: ImuReading = ImuReading {
    gyro_rps: [f32::NAN; 3],
    ..GOOD
};

#[test]
fn nan_secondary_gyro_does_not_take_over() {
    let mut mgr = DualImuManager::default_config();
    for _ in 0..5 {
        let sel = mgr.update_and_select(&GOOD, &NAN_GYRO, [0.01; 3]);
        assert_eq!(sel, GOOD);
    }
    assert_eq!(mgr.active_imu, ImuId::Primary);
    assert!(mgr.primary_healthy);
    assert!(!mgr.secondary_healthy);
}

#[test]
fn both_faulty_holds_active_imu() {
    let mut mgr = DualImuManager::default_config();
    for _ in 0..5 {
        mgr.update_and_select(&NAN_GYRO, &NAN_GYRO, [0.01; 3]);
    }
    assert_eq!(mgr.active_imu, ImuId::Primary);
    assert!(!mgr.primary_healthy);
    assert!(!mgr.secondary_healthy);
    mgr.update_and_select(&NAN_GYRO, &GOOD, [0.01; 3]);
    assert_eq!(mgr.active_imu, ImuId::Secondary);
}

#[test]
fn finite_disagreement_is_counted_against_primary() {
    let mut mgr = DualImuManager::default_config();
    let offset = ImuReading {
        gyro_rps: [2.5, -3.0, 1.8],
        ..GOOD
    };
    for _ in 0..3 {
        mgr.update_and_select(&GOOD, &offset, [0.01; 3]);
    }
    assert_eq!(mgr.active_imu, ImuId::Secondary);
    assert!(!mgr.primary_healthy);
}
