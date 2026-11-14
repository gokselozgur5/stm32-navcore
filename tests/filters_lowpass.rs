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
