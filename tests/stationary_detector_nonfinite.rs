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

#[test]
fn nonfinite_accel_window_is_rejected_and_not_stationary() {
    let mut detector = StationaryDetector::new(5, 0.05, 0.03);
    let mut stationary = false;
    for _ in 0..5 {
        stationary = detector.update([f32::NAN, 0.0, -9.80665], [0.0; 3]);
    }
    assert!(
        !stationary,
        "a window of NaN accel samples was reported as stationary"
    );
}

#[test]
fn nonfinite_or_off_gravity_magnitude_is_rejected_or_not_stationary() {
    // stationary_detector in ekf.rs does not check gravity magnitude (only variance),
    // whereas ZuptDetector checks deviation from gravity magnitude.
    // If accel is [0, 0, 0], variance is 0, so StationaryDetector incorrectly reports true!
    let mut detector = StationaryDetector::new(5, 0.05, 0.03);
    let mut stationary = false;
    for _ in 0..5 {
        stationary = detector.update([0.0, 0.0, 0.0], [0.0; 3]);
    }
    assert!(
        !stationary,
        "zero-g free fall must not be reported as stationary stance"
    );
}
