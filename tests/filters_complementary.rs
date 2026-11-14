use stm32_navcore::filters::{ComplementaryError, ComplementaryFilter};

#[test]
fn test_complementary_filter_validation() {
    assert_eq!(
        ComplementaryFilter::new(0.0, 0.01),
        Err(ComplementaryError::InvalidTimeConstant)
    );
    assert_eq!(
        ComplementaryFilter::new(-1.0, 0.01),
        Err(ComplementaryError::InvalidTimeConstant)
    );
    assert_eq!(
        ComplementaryFilter::new(1.0, 0.0),
        Err(ComplementaryError::InvalidSampleInterval)
    );
    assert_eq!(
        ComplementaryFilter::new(1.0, -0.01),
        Err(ComplementaryError::InvalidSampleInterval)
    );
}

#[test]
fn test_complementary_filter_initialization() {
    let mut filter = ComplementaryFilter::new(0.9, 0.1).unwrap();
    assert!((filter.alpha() - 0.9).abs() < 1e-6);
    let (pitch, roll) = filter.update(0.0, 0.0, 0.15, -0.25, 0.1);
    assert!((pitch - 0.15).abs() < 1e-6);
    assert!((roll - (-0.25)).abs() < 1e-6);
    assert!((filter.pitch() - 0.15).abs() < 1e-6);
    assert!((filter.roll() - (-0.25)).abs() < 1e-6);
}

#[test]
fn test_complementary_filter_fusion() {
    let mut filter = ComplementaryFilter::new(0.9, 0.1).unwrap();
    filter.update(0.0, 0.0, 0.0, 0.0, 0.1);
    // Gyro suggests 1.0 rad/s pitch, accel measures 0.0
    // gyro_pitch = 0.0 + 1.0*0.1 = 0.1
    // pitch = 0.9 * 0.1 + 0.1 * 0.0 = 0.09
    let (pitch, _) = filter.update(1.0, 0.0, 0.0, 0.0, 0.1);
    assert!((pitch - 0.09).abs() < 1e-5);
}
