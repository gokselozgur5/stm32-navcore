use stm32_navcore::filters::{LowPassError, NotchFilter};

#[test]
fn invalid_q_is_not_reported_as_invalid_sample_interval() {
    let err = NotchFilter::new(100.0, 0.001, 0.0).unwrap_err();
    assert_ne!(err, LowPassError::InvalidSampleInterval);
    assert_eq!(err, LowPassError::InvalidQ);
}

#[test]
fn negative_and_nan_q_reported_as_invalid_q() {
    assert_eq!(
        NotchFilter::new(100.0, 0.001, -1.0).unwrap_err(),
        LowPassError::InvalidQ
    );
    assert_eq!(
        NotchFilter::new(100.0, 0.001, f32::NAN).unwrap_err(),
        LowPassError::InvalidQ
    );
}
