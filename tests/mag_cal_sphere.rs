use stm32_navcore::sensors::mag_cal::SphereFit;

#[test]
fn recovers_known_sphere() {
    let c = [12.0f32, -7.0, 3.5];
    let r = 48.0f32;
    let mut f = SphereFit::new();
    for i in 0..8u32 {
        let d = |b: u32, k: usize| c[k] + r * if i & b != 0 { 0.57735 } else { -0.57735 };
        f.add([d(1, 0), d(2, 1), d(4, 2)]);
    }
    let (centre, radius) = f.solve().expect("fit");
    for i in 0..3 {
        assert!((centre[i] - c[i]).abs() < 1e-2, "axis {i}");
    }
    assert!((radius - r).abs() < 1e-2);
}

