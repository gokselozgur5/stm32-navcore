use stm32_navcore::math::Mat;

#[test]
fn exactly_singular_matrices_are_rejected() {
    assert!(Mat([[0.0]]).inv_1x1().is_none());
    assert!(Mat([[1.0, 2.0], [2.0, 4.0]]).inv_2x2().is_none());
    let rank2 = Mat([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    assert!(rank2.inv_3x3().is_none());
    assert!(Mat::<3, 3>::zeros().inv_3x3().is_none());
}
