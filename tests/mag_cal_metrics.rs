use stm32_navcore::sensors::mag_cal::{is_valid, MagCoverage, MagResidual};

fn full_coverage() -> MagCoverage {
    let mut c = MagCoverage::new();
    for i in 0..8u32 {
        let s = |b: u32| if i & b != 0 { 1.0 } else { -1.0 };
        c.add([s(1), s(2), s(4)], [0.0; 3]);
    }
    c
}

#[test]
fn coverage_counts_octants() {
    let mut c = MagCoverage::new();
    assert_eq!(c.octants(), 0);
    c.add([1.0, 1.0, 1.0], [0.0; 3]);
    c.add([2.0, 3.0, 4.0], [0.0; 3]);
    assert_eq!(c.octants(), 1);
    assert!((c.fraction() - 0.125).abs() < 1e-6);
    assert!(full_coverage().is_full());
}

