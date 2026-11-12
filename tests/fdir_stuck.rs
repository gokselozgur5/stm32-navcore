use stm32_navcore::fdir::StuckDetector;

#[test]
fn frozen_sample_is_flagged_after_limit() {
    let mut d = StuckDetector::new(3);
    let v = [0.1, -0.2, 0.3];
    assert!(!d.update(v));
    assert!(!d.update(v));
    assert!(!d.update(v));
    assert!(d.update(v));
}

#[test]
fn any_change_resets_the_counter() {
    let mut d = StuckDetector::new(2);
    let v = [1.0, 2.0, 3.0];
    d.update(v);
    d.update(v);
    assert!(!d.update([1.0, 2.0, 3.0001]));
    assert_eq!(d.repeats, 0);
    assert!(!d.update([1.0, 2.0, 3.0001]));
}
