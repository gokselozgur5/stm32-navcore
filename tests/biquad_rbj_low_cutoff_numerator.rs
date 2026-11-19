use stm32_navcore::filters::BiquadCoefficients;

// RBJ low-pass: b0 = (1 - cos w0) / (2 (1 + alpha)), w0 = 2 pi fc dt,
// alpha = sin w0 / (2 Q). For fc = 0.1 Hz, dt = 1 ms, Q = 1/sqrt(2) the
// value computed in f64 is 9.865221e-8.
#[test]
fn rbj_lowpass_numerator_survives_low_cutoff_ratio() {
    let q = core::f32::consts::FRAC_1_SQRT_2;
    if let Ok(c) = BiquadCoefficients::try_lowpass_rbj(0.1, 0.001, q) {
        let b0_ref = 9.865_221e-8_f32;
        let rel = ((c.b0 - b0_ref) / b0_ref).abs();
        assert!(rel < 0.01, "b0 = {:e}, expected {:e}", c.b0, b0_ref);
    }
    if let Ok(c) = BiquadCoefficients::try_lowpass_rbj(0.03, 0.001, q) {
        assert!(c.b0 > 0.0, "b0 collapsed to {}", c.b0);
    }
}
