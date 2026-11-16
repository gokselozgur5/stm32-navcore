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
    assert_eq!(
        f.try_update(f32::INFINITY),
        Err(ButterworthError::NonFinite)
    );
}

#[test]
fn test_state_reset_and_retune() {
    let mut f = ButterworthFilter::try_new(5.0, 0.01).expect("valid");
    for _ in 0..10 {
        f.update(1.0);
    }
    let (s1, s2) = f.state();
    assert!(s1 != 0.0 || s2 != 0.0);

    f.reset();
    assert_eq!(f.state(), (0.0, 0.0));

    f.set_state(0.5, -0.5);
    assert_eq!(f.state(), (0.5, -0.5));

    f.retune(15.0, 0.01).expect("retune success");
    assert!(f.coefficients().is_stable());
}

#[test]
fn test_batch_filter_matches_sequential() {
    let mut f_seq = ButterworthFilter::try_new(8.0, 0.005).expect("valid");
    let mut f_batch = ButterworthFilter::try_new(8.0, 0.005).expect("valid");

    let input = [1.0, 2.0, 1.5, -0.5, 0.0, 3.0, 2.5, 1.0];
    let mut out_batch = [0.0f32; 8];
    f_batch
        .batch_filter(&input, &mut out_batch)
        .expect("batch ok");

    for (idx, &x) in input.iter().enumerate() {
        let expected = f_seq.update(x);
        assert!((out_batch[idx] - expected).abs() < 1e-6);
    }

    let mut mismatched = [0.0f32; 5];
    assert_eq!(
        f_batch.batch_filter(&input, &mut mismatched),
        Err(ButterworthError::BufferLengthMismatch)
    );
}

#[test]
fn test_coefficients_construction_and_from_coeffs() {
    let coeffs = ButterworthCoefficients::try_new(5.0, 0.01).expect("valid coeffs");
    let filter = ButterworthFilter::from_coeffs(coeffs);
    assert_eq!(filter.coefficients(), &coeffs);
}
