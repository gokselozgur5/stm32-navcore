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

#[test]
fn test_cutoff_and_high_frequency_attenuation() {
    let dt = 0.001f32;
    let fc = 10.0f32;
    let mut f_cut = ButterworthFilter::try_new(fc, dt).expect("valid");
    let mut f_high = ButterworthFilter::try_new(fc, dt).expect("valid");

    let (mut max_cut, mut max_high) = (0.0f32, 0.0f32);
    for n in 0..2000 {
        let t = n as f32 * dt;
        let s_cut = libm::sinf(2.0 * core::f32::consts::PI * fc * t);
        let s_high = libm::sinf(2.0 * core::f32::consts::PI * 100.0 * t);
        let y_cut = f_cut.update(s_cut);
        let y_high = f_high.update(s_high);
        if n > 1500 {
            max_cut = max_cut.max(y_cut.abs());
            max_high = max_high.max(y_high.abs());
        }
    }
    assert!((max_cut - 0.7071).abs() < 0.05);
    assert!(max_high < 0.02);
}

#[test]
fn test_parameter_validation_and_errors() {
    assert_eq!(
        ButterworthFilter::try_new(0.0, 0.01),
        Err(ButterworthError::InvalidCutoffFrequency)
    );
    assert_eq!(
        ButterworthFilter::try_new(50.0, 0.01),
        Err(ButterworthError::InvalidCutoffFrequency)
    );
    assert_eq!(
        ButterworthFilter::try_new(10.0, 0.0),
        Err(ButterworthError::InvalidSampleInterval)
    );
    let mut f = ButterworthFilter::try_new(5.0, 0.01).expect("valid");
    assert_eq!(f.try_update(f32::NAN), Err(ButterworthError::NonFinite));
    assert_eq!(f.try_update(f32::INFINITY), Err(ButterworthError::NonFinite));
}
