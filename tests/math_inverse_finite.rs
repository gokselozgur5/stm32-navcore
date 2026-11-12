use stm32_navcore::math::Mat;

#[test]
fn subnormal_determinant_is_rejected() {
    let m2 = Mat([[1e-20, 0.0], [0.0, 1e-20]]);
    assert!(m2.inv_2x2().is_none(), "got {:?}", m2.inv_2x2());
    let m3 = Mat([[1e-13, 0.0, 0.0], [0.0, 1e-13, 0.0], [0.0, 0.0, 1e-13]]);
    assert!(m3.inv_3x3().is_none(), "got {:?}", m3.inv_3x3());
}

#[test]
fn overflowing_inverse_entry_is_rejected() {
    let skew = Mat([[1e-39, 0.0], [0.0, 1e38]]);
    assert!(skew.inv_2x2().is_none(), "got {:?}", skew.inv_2x2());
}
