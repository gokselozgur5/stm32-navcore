use stm32_navcore::math::Mat;

#[test]
fn exactly_singular_matrices_are_rejected() {
    assert!(Mat([[0.0]]).inv_1x1().is_none());
    assert!(Mat([[1.0, 2.0], [2.0, 4.0]]).inv_2x2().is_none());
    let rank2 = Mat([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    assert!(rank2.inv_3x3().is_none());
    assert!(Mat::<3, 3>::zeros().inv_3x3().is_none());
}

#[test]
fn non_finite_inputs_are_rejected() {
    assert!(Mat([[f32::NAN]]).inv_1x1().is_none());
    assert!(Mat([[f32::INFINITY]]).inv_1x1().is_none());
    assert!(Mat([[f32::NAN, 0.0], [0.0, 1.0]]).inv_2x2().is_none());
    let mut m = Mat::<3, 3>::identity();
    m.0[1][1] = f32::INFINITY;
    assert!(m.inv_3x3().is_none());
}

#[test]
fn near_singular_above_threshold_inverts_with_small_residual() {
    let m = Mat([[1.0, 1.0], [1.0, 1.0 + 1e-3]]);
    let inv = m.inv_2x2().expect("det 1e-3 is invertible");
    let prod = m.matmul(&inv);
    for r in 0..2 {
        for c in 0..2 {
            let want = if r == c { 1.0 } else { 0.0 };
            assert!((prod.0[r][c] - want).abs() < 5e-2);
        }
    }
}

#[test]
fn tiny_well_conditioned_inverts_and_tiny_singular_is_rejected() {
    assert!(Mat([[1e-13, 0.0], [0.0, 1e-13]]).inv_2x2().is_some());
    assert!(Mat([[1e-13, 2e-13], [2e-13, 4e-13]]).inv_2x2().is_none());
    let tiny3 = Mat([[1e-13, 0.0, 0.0], [0.0, 1e-13, 0.0], [0.0, 0.0, 1e-13]]);
    assert!(tiny3.inv_3x3().is_some());
    let rank2 = Mat([
        [1e-13, 2e-13, 3e-13],
        [4e-13, 5e-13, 6e-13],
        [7e-13, 8e-13, 9e-13],
    ]);
    assert!(rank2.inv_3x3().is_none());
    let ill = Mat([[1.0, 1.0], [1.0, 1.0 + 1e-7]]);
    assert!(ill.inv_2x2().is_none());
    assert!(Mat([[1e-13]]).inv_1x1().is_none());
}
