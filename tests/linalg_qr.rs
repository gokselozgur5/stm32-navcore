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
