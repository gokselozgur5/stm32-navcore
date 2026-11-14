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
