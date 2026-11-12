use stm32_navcore::math::Mat;

#[test]
fn tiny_diagonal_2x2_inverts() {
    let m = Mat([[1e-7, 0.0], [0.0, 1e-7]]);
    let inv = m.inv_2x2().expect("well conditioned tiny matrix");
    assert!((inv.0[0][0] - 1e7).abs() < 1.0);
    assert!((inv.0[1][1] - 1e7).abs() < 1.0);
    assert_eq!(inv.0[0][1], 0.0);
}

#[test]
fn tiny_scale_preserves_rejection_2x2() {
    let m = Mat([[1e-7, 2e-7], [2e-7, 4e-7]]);
    assert!(m.inv_2x2().is_none());
}

#[test]
fn large_nearly_singular_2x2_is_rejected() {
    let m = Mat([[1e6, 1e6], [1e6, 1e6 + 1.0]]);
    assert!(m.inv_2x2().is_none());
}

#[test]
fn large_well_conditioned_2x2_inverts() {
    let m = Mat([[1e6, 0.0], [0.0, 2e6]]);
    let inv = m.inv_2x2().expect("well conditioned");
    assert!((inv.0[1][1] - 5e-7).abs() < 1e-12);
}
