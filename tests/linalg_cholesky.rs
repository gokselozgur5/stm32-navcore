use stm32_navcore::linalg::cholesky::cholesky;
use stm32_navcore::math::Mat;

fn spd<const N: usize>() -> Mat<N, N> {
    let mut b = Mat::<N, N>::zeros();
    for (i, row) in b.0.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = ((i * 7 + j * 3) % 5) as f32 * 0.3 - 0.4;
        }
    }
    b.matmul(&b.transpose()).add(&Mat::<N, N>::identity())
}

fn max_diff<const N: usize>(a: &Mat<N, N>, b: &Mat<N, N>) -> f32 {
    a.sub(b).0.iter().flatten().fold(0.0, |m, v| m.max(v.abs()))
}

fn residual<const N: usize>() -> f32 {
    let a = spd::<N>();
    let l = cholesky(&a).unwrap();
    max_diff(&l.matmul(&l.transpose()), &a)
}

#[test]
fn llt_reproduces_spd_3x3() {
    assert!(residual::<3>() < 1e-5);
}

#[test]
fn llt_reproduces_spd_6x6() {
    assert!(residual::<6>() < 1e-5);
}

#[test]
fn rejects_non_pd_and_non_finite() {
    use stm32_navcore::linalg::LinalgError;
    let mut a = spd::<3>();
    a.0[1][1] = -1.0;
    assert_eq!(cholesky(&a), Err(LinalgError::NotPositiveDefinite));
    let mut b = spd::<3>();
    b.0[2][0] = f32::NAN;
    assert_eq!(cholesky(&b), Err(LinalgError::NonFinite));
    b.0[2][0] = f32::INFINITY;
    assert_eq!(cholesky(&b), Err(LinalgError::NonFinite));
}
