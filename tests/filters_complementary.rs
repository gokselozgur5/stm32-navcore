use stm32_navcore::filters::{ComplementaryError, ComplementaryFilter};

#[test]
fn test_complementary_filter_construction_and_step() {
    let mut filter = ComplementaryFilter::new(0.98, 0.01).unwrap();
    assert_eq!(filter.roll(), 0.0);
    assert_eq!(filter.pitch(), 0.0);

    filter.update(0.1, -0.1, 0.05, -0.05);
    assert!(filter.roll() > 0.0);
    assert!(filter.pitch() < 0.0);

    filter.reset();
    assert_eq!(filter.roll(), 0.0);
    assert_eq!(filter.pitch(), 0.0);
}

#[test]
fn test_complementary_filter_invalid_parameters() {
    assert_eq!(
        ComplementaryFilter::new(0.98, 0.0),
        Err(ComplementaryError::InvalidSampleInterval)
    );
    assert_eq!(
        ComplementaryFilter::new(1.0, 0.01),
        Err(ComplementaryError::InvalidAlpha)
    );
    assert_eq!(
        ComplementaryFilter::new(0.0, 0.01),
        Err(ComplementaryError::InvalidAlpha)
    );
}
