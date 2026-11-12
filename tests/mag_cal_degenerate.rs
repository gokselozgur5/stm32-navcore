use stm32_navcore::sensors::mag_cal::MagMinMax;

fn fed(z: f32) -> MagMinMax {
    let mut mm = MagMinMax::new();
    for s in [[40.0, 0.0, z], [-40.0, 0.0, -z], [0.0, 40.0, 0.0], [0.0, -40.0, 0.0]] {
        mm.update(s);
    }
    mm
}

#[test]
fn zero_span_axis_does_not_distort_other_gains() {
    let g = fed(0.0).scale();
    assert!((g[0] - 1.0).abs() < 1e-6 && (g[1] - 1.0).abs() < 1e-6 && g[2] == 1.0);
}

#[test]
fn under_excited_axis_keeps_unit_gain() {
    let g = fed(1.0).scale();
    assert!((g[0] - 1.0).abs() < 1e-6 && (g[1] - 1.0).abs() < 1e-6 && g[2] == 1.0);
}
