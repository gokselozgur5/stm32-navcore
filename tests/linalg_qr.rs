use stm32_navcore::linalg::qr::qr;
use stm32_navcore::math::Mat;

#[test]
fn qr_reconstructs_and_orthonormal_6x4() {
    let mut a = Mat::<6, 4>::zeros();
    for (i, row) in a.0.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = ((i * 5 + j * 7 + 3) % 11) as f32 * 0.5 - 2.5;
        }
    }
    let (q, r) = qr(&a).unwrap();
    let qr_prod = q.matmul(&r);
    for i in 0..6 {
        for j in 0..4 {
            assert!((qr_prod.0[i][j] - a.0[i][j]).abs() < 1e-5);
        }
    }
    let qt_q = q.transpose().matmul(&q);
    let eye = Mat::<6, 6>::identity();
    for i in 0..6 {
        for j in 0..6 {
            assert!((qt_q.0[i][j] - eye.0[i][j]).abs() < 1e-5);
        }
    }
}

#[test]
fn qr_square_3x3_and_non_finite() {
    use stm32_navcore::linalg::qr::Givens;
    use stm32_navcore::linalg::LinalgError;
    let g = Givens::compute(3.0, 4.0);
    assert!((g.r - 5.0).abs() < 1e-6);
    assert!((g.c - 0.6).abs() < 1e-6);
    assert!((g.s - 0.8).abs() < 1e-6);
    let a = Mat([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]]);
    let (q, r) = qr(&a).unwrap();
    let diff = q.matmul(&r).sub(&a);
    for row in diff.0.iter() {
        for &v in row.iter() {
            assert!(v.abs() < 1e-5);
        }
    }
    let mut bad = a;
    bad.0[0][0] = f32::NAN;
    assert_eq!(qr(&bad), Err(LinalgError::NonFinite));
}
