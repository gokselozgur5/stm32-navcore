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

fn assert_identity<const N: usize>(m: &Mat<N, N>, inv: &Mat<N, N>, tol: f32) {
    let prod = m.matmul(inv);
    for r in 0..N {
        for c in 0..N {
            let want = if r == c { 1.0 } else { 0.0 };
            assert!((prod.0[r][c] - want).abs() < tol, "{:?}", prod);
        }
    }
}

#[test]
fn tiny_and_large_scale_inverses_are_correct() {
    let d7 = Mat([[1e-7, 0.0], [0.0, 1e-7]]);
    let inv = d7.inv_2x2().expect("diag(1e-7) is invertible");
    assert!((inv.0[0][0] - 1e7).abs() < 1.0 && inv.0[0][1] == 0.0);
    assert_identity(&d7, &inv, 1e-5);
    let tiny = Mat([[2e-13, 1e-13], [1e-13, 3e-13]]);
    assert_identity(&tiny, &tiny.inv_2x2().expect("tiny 2x2"), 1e-5);
    let big = Mat([[4e12, 1e12, 0.0], [1e12, 3e12, 1e12], [0.0, 1e12, 2e12]]);
    assert_identity(&big, &big.inv_3x3().expect("large 3x3"), 1e-5);
    let small3 = Mat([[3e-6, 1e-6, 0.0], [1e-6, 2e-6, 0.0], [0.0, 0.0, 1e-6]]);
    assert_identity(&small3, &small3.inv_3x3().expect("small 3x3"), 1e-5);
}
