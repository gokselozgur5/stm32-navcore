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
