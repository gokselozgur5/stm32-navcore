use stm32_navcore::filters::Biquad;

#[test]
fn dc_gain_is_unity_and_nyquist_attenuation_exceeds_40db() {
    let cutoff_hz = 50.0;
    let dt_s = 0.001;
    let q = 1.0 / libm::sqrtf(2.0);
    let mut bq = Biquad::lowpass_rbj(cutoff_hz, dt_s, q).unwrap();
    let (b0, b1, b2, a1, a2) = bq.coefficients();
    let dc_gain = (b0 + b1 + b2) / (1.0 + a1 + a2);
    assert!((dc_gain - 1.0).abs() < 1e-5);
    for n in 0..200 {
        let x = if n % 2 == 0 { 1.0 } else { -1.0 };
        let y = bq.update(x);
        if n >= 150 {
            assert!(y.abs() < 0.01);
        }
    }
}
