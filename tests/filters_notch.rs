use stm32_navcore::filters::NotchFilter;

#[test]
fn center_frequency_attenuated_over_30db() {
    let f0 = 100.0;
    let dt_s = 0.001;
    let q = 2.0;
    let mut notch = NotchFilter::new(f0, dt_s, q).unwrap();
    let mut max_amp = 0.0f32;
    for n in 0..600 {
        let t = n as f32 * dt_s;
        let x = libm::sinf(2.0 * core::f32::consts::PI * f0 * t);
        let y = notch.update(x);
        if n >= 400 && y.abs() > max_amp {
            max_amp = y.abs();
        }
    }
    let atten_db = -20.0 * libm::log10f(max_amp);
    assert!(atten_db > 30.0);
}

#[test]
fn octave_tone_passes_within_one_db() {
    let f0 = 100.0;
    let dt_s = 0.001;
    let q = 2.0;
    let mut notch = NotchFilter::new(f0, dt_s, q).unwrap();
    let f_octave = 200.0;
    let mut max_amp = 0.0f32;
    for n in 0..600 {
        let t = n as f32 * dt_s;
        let x = libm::sinf(2.0 * core::f32::consts::PI * f_octave * t);
        let y = notch.update(x);
        if n >= 400 && y.abs() > max_amp {
            max_amp = y.abs();
        }
    }
    let gain_db = 20.0 * libm::log10f(max_amp);
    assert!(gain_db.abs() < 1.0);
}

#[test]
fn reject_invalid_sample_interval_before_nyquist() {
    use stm32_navcore::filters::LowPassError;
    let err = NotchFilter::new(100.0, -0.001, 2.0).unwrap_err();
    assert_eq!(err, LowPassError::InvalidSampleInterval);
}

#[test]
fn dynamic_retune_attenuates_new_frequency() {
    let mut notch = NotchFilter::new(100.0, 0.001, 2.0).unwrap();
    notch.set_frequency(250.0, 0.001, 2.0).unwrap();
    let mut max_amp = 0.0f32;
    for n in 0..600 {
        let t = n as f32 * 0.001;
        let x = libm::sinf(2.0 * core::f32::consts::PI * 250.0 * t);
        let y = notch.update(x);
        if n >= 400 && y.abs() > max_amp {
            max_amp = y.abs();
        }
    }
    let atten_db = -20.0 * libm::log10f(max_amp);
    assert!(atten_db > 30.0);
}

#[test]
fn notch_coefficients_accessor_matches_underlying_biquad() {
    let notch = NotchFilter::new(100.0, 0.001, 2.0).unwrap();
    let coeffs = notch.coefficients();
    assert_eq!(coeffs, notch.biquad().coefficients());
}

#[test]
fn notch_preserves_dc_unity_gain() {
    let mut notch = NotchFilter::new(150.0, 0.001, 2.0).unwrap();
    let mut out = 0.0f32;
    for _ in 0..1000 {
        out = notch.update(1.0);
    }
    assert!((out - 1.0).abs() < 1e-4);
}

#[test]
fn reject_invalid_frequency_in_retune() {
    use stm32_navcore::filters::LowPassError;
    let mut notch = NotchFilter::new(100.0, 0.001, 2.0).unwrap();
    assert_eq!(
        notch.set_frequency(-10.0, 0.001, 2.0).unwrap_err(),
        LowPassError::InvalidCutoffFrequency
    );
    assert_eq!(
        notch.set_frequency(600.0, 0.001, 2.0).unwrap_err(),
        LowPassError::InvalidCutoffFrequency
    );
}
