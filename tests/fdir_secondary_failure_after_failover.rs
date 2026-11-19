use stm32_navcore::fdir::{DualImuManager, ImuReading};

#[test]
fn finite_primary_is_selected_when_secondary_dies_after_failover() {
    let good = ImuReading { accel_mps2: [0.0, 0.0, -9.81], gyro_rps: [0.0; 3] };
    let spike = ImuReading { gyro_rps: [2.5, -3.0, 1.8], ..good };
    let dead = ImuReading { accel_mps2: [f32::NAN; 3], gyro_rps: [f32::NAN; 3] };
    let mut mgr = DualImuManager::default_config().with_recovery(5);
    for _ in 0..3 { mgr.update_and_select(&spike, &good, [0.01; 3]); }
    let mut selected = good;
    for _ in 0..100 { selected = mgr.update_and_select(&good, &dead, [0.01; 3]); }
    let finite = selected.gyro_rps.iter().chain(selected.accel_mps2.iter()).all(|v| v.is_finite());
    assert!(finite, "selected {selected:?} while the primary is finite");
}
