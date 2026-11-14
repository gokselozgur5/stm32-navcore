use stm32_navcore::filters::{BiquadCoefficients, BiquadFilter};

#[test]
fn test_biquad_dc_gain_and_reset() {
    let coeffs = BiquadCoefficients::lowpass_rbj(10.0, 0.001, core::f32::consts::FRAC_1_SQRT_2);
    let mut filter = BiquadFilter::new(coeffs);
    let mut out = 0.0;
    for _ in 0..1000 {
        out = filter.update(1.0);
    }
    assert!((out - 1.0).abs() < 1e-3);

    filter.reset();
    let initial_step = filter.update(1.0);
    assert!((initial_step - 1.0).abs() > 0.01);
}

#[test]
fn test_biquad_cutoff_attenuation() {
    let dt = 0.001f32;
    let fc = 20.0f32;
    let q = core::f32::consts::FRAC_1_SQRT_2;
    let coeffs = BiquadCoefficients::lowpass_rbj(fc, dt, q);
    let mut filter = BiquadFilter::new(coeffs);

    let mut peak = 0.0f32;
    for i in 0..4000 {
        let t = i as f32 * dt;
        let x = libm::sinf(2.0 * core::f32::consts::PI * fc * t);
        let y = filter.update(x);
        if i >= 2000 && y.abs() > peak {
            peak = y.abs();
        }
    }
    let gain_db = 20.0 * libm::log10f(peak);
    assert!((gain_db - (-3.01)).abs() < 0.2);
}

#[test]
fn test_biquad_nyquist_attenuation() {
    let dt = 0.001f32;
    let fc = 10.0f32;
    let q = core::f32::consts::FRAC_1_SQRT_2;
    let coeffs = BiquadCoefficients::lowpass_rbj(fc, dt, q);
    let mut filter = BiquadFilter::new(coeffs);

    let mut peak = 0.0f32;
    for i in 0..2000 {
        let x = if i % 2 == 0 { 1.0f32 } else { -1.0f32 };
        let y = filter.update(x);
        if i >= 1000 && y.abs() > peak {
            peak = y.abs();
        }
    }
    let gain_db = 20.0 * libm::log10f(peak);
    assert!(gain_db < -40.0);
}
