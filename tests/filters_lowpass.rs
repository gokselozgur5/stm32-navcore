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

#[test]
fn step_response_reaches_63_percent_at_one_time_constant() {
    let cutoff_hz = 10.0;
    let dt_s = 0.001;
    let mut filter = LowPassFilter::from_cutoff(cutoff_hz, dt_s).unwrap();
    let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
    let n_steps = (tau / dt_s + 0.5) as usize;
    let mut output = filter.update(0.0);
    for _ in 0..n_steps {
        output = filter.update(1.0);
    }
    let expected = 0.632;
    assert!((output - expected).abs() < 0.02);
}
