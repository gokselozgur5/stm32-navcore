use stm32_navcore::filters::{FilterError, LowPassFilter};

#[test]
fn invalid_cutoff_and_dt_are_rejected() {
    assert_eq!(
        LowPassFilter::from_cutoff(-5.0, 0.01),
        Err(FilterError::InvalidCutoff)
    );
    assert_eq!(
        LowPassFilter::from_cutoff(0.0, 0.01),
        Err(FilterError::InvalidCutoff)
    );
    assert_eq!(
        LowPassFilter::from_cutoff(f32::NAN, 0.01),
        Err(FilterError::NonFinite)
    );
    assert_eq!(
        LowPassFilter::from_cutoff(10.0, -0.01),
        Err(FilterError::InvalidDt)
    );
    assert_eq!(
        LowPassFilter::from_cutoff(60.0, 0.01),
        Err(FilterError::AboveNyquist)
    );
}
