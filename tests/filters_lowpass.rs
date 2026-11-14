use stm32_navcore::filters::LowPassFilter;

#[test]
fn test_rejection_of_invalid_cutoff_and_dt() {
    assert!(LowPassFilter::new(0.0, 0.01).is_err());
    assert!(LowPassFilter::new(-5.0, 0.01).is_err());
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
