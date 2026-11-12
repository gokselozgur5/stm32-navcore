use stm32_navcore::sensors::baro::BaroFilter;

#[test]
fn sustained_step_settles_quickly_after_forced_acceptance() {
    let mut f = BaroFilter::new(0.3, 5.0, 3);
    for _ in 0..20 {
        f.update(0.0);
    }
    let mut settled = None;
    for i in 1..=60 {
        if let Some(v) = f.update(100.0) {
            if (v - 100.0).abs() < 1.0 {
                settled = Some(i);
                break;
            }
        }
    }
    let n = settled.expect("never settled");
    assert!(n <= 14, "settled after {n} samples");
}
