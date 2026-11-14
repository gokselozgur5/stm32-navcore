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
