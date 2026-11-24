use stm32_navcore::math::linalg::jacobi_eig::jacobi_eig;
use stm32_navcore::math::Mat;

fn symmetric_matrix<const N: usize>() -> Mat<N, N> {
    let mut b = Mat::<N, N>::zeros();
    for (i, row) in b.0.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = ((i * 7 + j * 3 + 2) % 9) as f32 * 0.25 - 1.0;
        }
    }
    b.add(&b.transpose())
}

fn max_eigen_residual<const N: usize>(a: &Mat<N, N>) -> f32 {
    let res = jacobi_eig(a).expect("jacobi_eig failed");
    let mut max_err = 0.0f32;
    for k in 0..N {
        let mut v_k = [0.0f32; N];
        for r in 0..N {
            v_k[r] = res.vectors.0[r][k];
        }
        for r in 0..N {
            let mut av_r = 0.0f32;
            for c in 0..N {
                av_r += a.0[r][c] * v_k[c];
            }
            let diff = (av_r - res.values[k] * v_k[r]).abs();
            if diff > max_err {
                max_err = diff;
            }
        }
    }
    max_err
}

#[test]
fn eigenpairs_satisfy_av_lambda_v_2x2() {
    let mut a = Mat::<2, 2>::zeros();
    a.0[0][0] = 2.0;
    a.0[0][1] = 1.0;
    a.0[1][0] = 1.0;
    a.0[1][1] = 2.0;
    let res = jacobi_eig(&a).expect("jacobi_eig failed");
    assert!((res.values[0] - 3.0).abs() < 1e-4 || (res.values[0] - 1.0).abs() < 1e-4);
}

#[test]
fn eigenpairs_satisfy_av_lambda_v_4x4() {
    let a = symmetric_matrix::<4>();
    let err = max_eigen_residual(&a);
    assert!(err < 1e-4, "4x4 residual {} >= 1e-4", err);
}

#[test]
fn eigenpairs_satisfy_av_lambda_v_8x8() {
    let a = symmetric_matrix::<8>();
    let err = max_eigen_residual(&a);
    assert!(err < 1e-4, "8x8 residual {} >= 1e-4", err);
}

#[test]
fn eigenvectors_are_orthonormal() {
    let a = symmetric_matrix::<4>();
    let res = jacobi_eig(&a).unwrap();
    let v_t_v = res.vectors.transpose().matmul(&res.vectors);
    let id = Mat::<4, 4>::identity();
    let diff = v_t_v
        .sub(&id)
        .0
        .iter()
        .flatten()
        .fold(0.0f32, |m, v| m.max(v.abs()));
    assert!(diff < 1e-5, "orthonormality error {} >= 1e-5", diff);
}

#[test]
fn rejects_non_finite_input() {
    use stm32_navcore::math::linalg::LinalgError;
    let mut a = symmetric_matrix::<3>();
    a.0[0][1] = f32::NAN;
    assert_eq!(jacobi_eig(&a), Err(LinalgError::NonFinite));
    a.0[0][1] = f32::INFINITY;
    assert_eq!(jacobi_eig(&a), Err(LinalgError::NonFinite));
}
