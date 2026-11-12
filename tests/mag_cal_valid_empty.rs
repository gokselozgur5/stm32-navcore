use stm32_navcore::sensors::mag_cal::{is_valid, MagCoverage, MagResidual};

#[test]
fn empty_residual_is_not_valid() {
    let mut c = MagCoverage::new();
    for i in 0..8u32 {
        let s = |b: u32| if i & b != 0 { 1.0 } else { -1.0 };
        c.add([s(1), s(2), s(4)], [0.0; 3]);
    }
    assert!(c.is_full());
    assert!(!is_valid(&c, &MagResidual::default(), 50.0, 0.05));
}
