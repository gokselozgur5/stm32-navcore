use stm32_navcore::sensors::mag_cal::{MagCal, MagMinMax};

fn fill() -> MagMinMax {
    let mut mm = MagMinMax::new();
    for s in [
        [30.0, 5.0, -20.0],
        [-10.0, 5.0, -20.0],
        [10.0, 45.0, -20.0],
        [10.0, -35.0, -20.0],
        [10.0, 5.0, 20.0],
        [10.0, 5.0, -60.0],
    ] {
        mm.update(s);
    }
    mm
}

#[test]
fn offset_is_midpoint() {
    assert_eq!(fill().offset(), [10.0, 5.0, -20.0]);
    assert_eq!(MagMinMax::new().offset(), [0.0; 3]);
}

#[test]
fn scale_equalises_half_ranges() {
    let s = fill().scale();
    assert!((s[0] - 5.0 / 3.0).abs() < 1e-5);
    assert!((s[1] - 5.0 / 6.0).abs() < 1e-5);
    assert!((s[2] - 5.0 / 6.0).abs() < 1e-5);
    assert_eq!(MagMinMax::new().scale(), [1.0; 3]);
}

#[test]
fn apply_corrects_extremes_to_unit_radius() {
    let cal = MagCal::from_min_max(&fill());
    let c = cal.apply([30.0, 5.0, -20.0]);
    assert!((c[0] - 100.0 / 3.0).abs() < 1e-4);
    assert_eq!((c[1], c[2]), (0.0, 0.0));
    assert_eq!(MagCal::IDENTITY.apply([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
}
