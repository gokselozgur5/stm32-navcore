use stm32_navcore::StationaryDetector;

#[test]
fn nan_gyro_window_is_not_stationary() {
    let mut detector = StationaryDetector::new(5, 0.05, 0.03);
    let mut stationary = false;
    for _ in 0..5 {
        stationary = detector.update([0.0, 0.0, -9.80665], [f32::NAN; 3]);
    }
    assert!(
        !stationary,
        "a window of NaN gyro samples was reported as stationary"
    );
}
