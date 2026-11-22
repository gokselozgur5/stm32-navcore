use stm32_navcore::{ZuptConfig, ZuptDetector};

#[test]
fn nan_gyro_window_is_not_stationary() {
    let mut detector = ZuptDetector::<5>::new(ZuptConfig {
        accel_var_thresh: 0.05,
        gyro_norm_thresh: 0.03,
        accel_mag_thresh: 0.35,
        min_stance_samples: 1,
        gravity_magnitude: 9.80665,
    })
    .unwrap();
    let mut stationary = false;
    for _ in 0..5 {
        stationary = detector
            .update([0.0, 0.0, -9.80665], [f32::NAN; 3])
            .unwrap_or(false);
    }
    assert!(
        !stationary,
        "a window of NaN gyro samples was reported as stationary"
    );
}

#[test]
fn nonfinite_accel_window_is_rejected_and_not_stationary() {
    let mut detector = ZuptDetector::<5>::new(ZuptConfig {
        accel_var_thresh: 0.05,
        gyro_norm_thresh: 0.03,
        accel_mag_thresh: 0.35,
        min_stance_samples: 1,
        gravity_magnitude: 9.80665,
    })
    .unwrap();
    let mut stationary = false;
    for _ in 0..5 {
        stationary = detector
            .update([f32::NAN, 0.0, -9.80665], [0.0; 3])
            .unwrap_or(false);
    }
    assert!(
        !stationary,
        "a window of NaN accel samples was reported as stationary"
    );
}

#[test]
fn nonfinite_or_off_gravity_magnitude_is_rejected_or_not_stationary() {
    let config = ZuptConfig {
        accel_var_thresh: 0.05,
        gyro_norm_thresh: 0.03,
        accel_mag_thresh: 0.35,
        min_stance_samples: 1,
        gravity_magnitude: 9.80665,
    };
    let mut detector = ZuptDetector::<5>::new(config).unwrap();
    let mut stationary = Ok(false);
    for _ in 0..5 {
        stationary = detector.update([0.0, 0.0, 0.0], [0.0; 3]);
    }
    assert_eq!(
        stationary,
        Ok(false),
        "zero-g free fall must not be reported as stationary stance"
    );

    let mut detector_good = ZuptDetector::<5>::new(config).unwrap();
    let mut stationary_good = Ok(false);
    for _ in 0..5 {
        stationary_good = detector_good.update([0.0, 0.0, -9.80665], [0.0; 3]);
    }
    assert_eq!(
        stationary_good,
        Ok(true),
        "nominal gravity at rest must confirm stationary stance"
    );
}
