use stm32_navcore::butterworth::{ButterworthCoefficients, ButterworthError, ButterworthFilter};

#[test]
fn test_dc_unity_gain_step_response() {
    let mut filter = ButterworthFilter::try_new(5.0, 0.01).expect("valid filter");
    assert!(filter.coefficients().is_stable());

    let mut out = 0.0;
    for _ in 0..500 {
        out = filter.update(10.0);
    }
    assert!((out - 10.0).abs() < 1e-3);
}
