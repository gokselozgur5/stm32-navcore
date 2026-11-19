use stm32_navcore::filters::ComplementaryFilter;

#[test]
fn roll_crossing_the_pi_seam_takes_the_short_way() {
    let mut f = ComplementaryFilter::new(0.98, 0.01).unwrap();
    for _ in 0..2000 {
        f.update(0.0, 0.0, 3.13, 0.0);
    }
    assert!((f.roll() - 3.13).abs() < 1.0e-3);
    let mut min_abs = f32::MAX;
    for _ in 0..200 {
        f.update(0.0, 0.0, -3.13, 0.0);
        min_abs = min_abs.min(f.roll().abs());
    }
    assert!(min_abs > 3.0, "estimate passed through |roll| = {min_abs}");
}
