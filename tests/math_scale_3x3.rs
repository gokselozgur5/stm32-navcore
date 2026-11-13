use stm32_navcore::math::Mat;

#[test]
fn tiny_diagonal_3x3_inverts() {
    let m = Mat([[1e-7, 0.0, 0.0], [0.0, 1e-7, 0.0], [0.0, 0.0, 1e-7]]);
    let inv = m.inv_3x3().expect("well conditioned tiny matrix");
    for (i, row) in inv.0.iter().enumerate() {
        assert!((row[i] - 1e7).abs() < 1.0);
    }
}

#[test]
fn large_nearly_singular_3x3_is_rejected() {
    let m = Mat([[1e6, 1e6, 1e6], [1e6, 1e6, 1e6], [1e6, 1e6, 1e6 + 1.0]]);
    assert!(m.inv_3x3().is_none());
}
