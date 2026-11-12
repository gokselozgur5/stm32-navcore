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

#[test]
fn residual_rms_and_relative() {
    let mut r = MagResidual::new();
    assert_eq!(r.rms(), 0.0);
    r.add([11.0, 0.0, 0.0], 10.0);
    r.add([0.0, 9.0, 0.0], 10.0);
    assert!((r.rms() - 1.0).abs() < 1e-6);
    assert!((r.relative(10.0) - 0.1).abs() < 1e-6);
    assert!(r.relative(0.0).is_infinite());
}

#[test]
fn validity_needs_coverage_and_low_residual() {
    let mut r = MagResidual::new();
    r.add([10.1, 0.0, 0.0], 10.0);
    assert!(is_valid(&full_coverage(), &r, 10.0, 0.05));
    assert!(!is_valid(&MagCoverage::new(), &r, 10.0, 0.05));
    assert!(!is_valid(&full_coverage(), &r, 10.0, 0.001));
}
