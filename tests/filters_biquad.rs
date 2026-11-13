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

#[test]
fn gain_at_cutoff_is_minus_three_db() {
    let cutoff_hz = 100.0;
    let dt_s = 0.001;
    let q = 1.0 / libm::sqrtf(2.0);
    let bq = Biquad::lowpass_rbj(cutoff_hz, dt_s, q).unwrap();
    let (b0, b1, b2, a1, a2) = bq.coefficients();
    let w0 = 2.0 * core::f32::consts::PI * cutoff_hz * dt_s;
    let re_num = b0 + b1 * libm::cosf(w0) + b2 * libm::cosf(2.0 * w0);
    let im_num = -(b1 * libm::sinf(w0) + b2 * libm::sinf(2.0 * w0));
    let re_den = 1.0 + a1 * libm::cosf(w0) + a2 * libm::cosf(2.0 * w0);
    let im_den = -(a1 * libm::sinf(w0) + a2 * libm::sinf(2.0 * w0));
    let mag_sq = (re_num * re_num + im_num * im_num) / (re_den * re_den + im_den * im_den);
    let gain_db = 10.0 * libm::log10f(mag_sq);
    assert!((gain_db - (-3.01)).abs() < 0.2);
}
