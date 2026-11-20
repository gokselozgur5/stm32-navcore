use stm32_navcore::filters::{LowPassError, LowPassFilter};

#[test]
fn test_rejection_of_invalid_cutoff_and_dt() {
    assert_eq!(
        LowPassFilter::new(0.0, 0.01),
        Err(LowPassError::InvalidCutoffFrequency)
    );
    assert_eq!(
        LowPassFilter::new(-5.0, 0.01),
        Err(LowPassError::InvalidCutoffFrequency)
    );
    assert!(LowPassFilter::new(50.0, 0.01).is_err());
    assert!(LowPassFilter::new(60.0, 0.01).is_err());
    assert!(LowPassFilter::new(10.0, 0.0).is_err());
    assert!(LowPassFilter::new(10.0, -0.01).is_err());
}

#[test]
fn test_filter_step_response_and_convergence() {
    let mut f = LowPassFilter::new(10.0, 0.01).expect("valid parameters");
    assert_eq!(f.update(100.0), 100.0);
    let mut val = 100.0;
    for _ in 0..100 {
        val = f.update(0.0);
    }
    assert!(val < 1.0);
}

#[test]
fn test_filter_reset_reinitializes_on_next_sample() {
    let mut f = LowPassFilter::new(10.0, 0.01).expect("valid parameters");
    assert_eq!(f.update(50.0), 50.0);
    f.update(10.0);
    f.reset();
    assert_eq!(f.update(25.0), 25.0);
}

#[test]
fn test_step_response_at_one_time_constant() {
    let cutoff_hz = 10.0;
    let dt = 0.001;
    let mut f = LowPassFilter::from_cutoff(cutoff_hz, dt).unwrap();
    let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
    let n_steps = (tau / dt + 0.5) as usize;
    let _ = f.update(0.0);
    let mut out = 0.0;
    for _ in 0..n_steps {
        out = f.update(1.0);
    }
    let expected = 0.632_f32;
    assert!((out - expected).abs() < 0.02);
}

#[test]
fn test_filter_state_tracking_and_error_variants() {
    use stm32_navcore::filters::LowPassError;
    let mut f = LowPassFilter::new(10.0, 0.01).unwrap();
    assert_eq!(f.state(), None);
    assert_eq!(f.update(42.0), 42.0);
    assert_eq!(f.state(), Some(42.0));
    f.reset();
    assert_eq!(f.state(), None);
    assert_eq!(
        LowPassFilter::new(f32::NAN, 0.01),
        Err(LowPassError::InvalidCutoffFrequency)
    );
    assert_eq!(
        LowPassFilter::new(10.0, f32::NAN),
        Err(LowPassError::InvalidSampleInterval)
    );
}
