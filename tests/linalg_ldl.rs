use stm32_navcore::linalg::ldl::{ldl, ldl_det, ldl_log_det, ldl_reconstruct, ldl_solve};
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

#[test]
fn factor_has_unit_diagonal_and_positive_d() {
    let (l, d) = ldl(&spd::<4>()).unwrap();
    for (i, di) in d.iter().enumerate() {
        assert_eq!(l.0[i][i], 1.0);
        assert!(*di > 0.0);
    }
}

fn residual<const N: usize>() -> f32 {
    let a = spd::<N>();
    let (l, d) = ldl(&a).unwrap();
    let mut dm = Mat::<N, N>::zeros();
    for (i, di) in d.iter().enumerate() {
        dm.0[i][i] = *di;
    }
    max_diff(&l.matmul(&dm).matmul(&l.transpose()), &a)
}

#[test]
fn ldlt_reproduces_spd_input() {
    assert!(residual::<3>() < 1e-5);
    assert!(residual::<6>() < 1e-4);
}

#[test]
fn non_positive_pivot_is_reported() {
    use stm32_navcore::linalg::LinalgError;
    let mut a = spd::<3>();
    a.0[1][1] = -0.5;
    assert_eq!(ldl(&a), Err(LinalgError::NotPositiveDefinite));
    a.0[1][1] = f32::NAN;
    assert_eq!(ldl(&a), Err(LinalgError::NonFinite));
}

#[test]
fn ldl_reconstruct_reproduces_spd() {
    let a = spd::<4>();
    let (l, d) = ldl(&a).unwrap();
    let recon = ldl_reconstruct(&l, &d);
    assert!(max_diff(&recon, &a) < 1e-5);
}

#[test]
fn ldl_det_computes_spd_determinant() {
    let a = spd::<3>();
    let (_, d) = ldl(&a).unwrap();
    let det = ldl_det(&d);
    assert!(det > 1.0);
}

#[test]
fn ldl_log_det_matches_log_of_det() {
    let a = spd::<3>();
    let (_, d) = ldl(&a).unwrap();
    let log_det = ldl_log_det(&d);
    let expected = libm::logf(ldl_det(&d));
    assert!((log_det - expected).abs() < 1e-4);
}

#[test]
fn ldl_solve_reproduces_solution() {
    let a = spd::<3>();
    let (l, d) = ldl(&a).unwrap();
    let b = [1.0f32, -2.0, 3.0];
    let x = ldl_solve(&l, &d, &b).unwrap();
    let mut ax = [0.0f32; 3];
    for i in 0..3 {
        for j in 0..3 {
            ax[i] += a.0[i][j] * x[j];
        }
    }
    for i in 0..3 {
        assert!((ax[i] - b[i]).abs() < 1e-4);
    }
}

#[test]
fn ldl_reads_only_lower_triangle() {
    let mut a = spd::<3>();
    let (l_clean, d_clean) = ldl(&a).unwrap();
    a.0[0][1] = 999.0;
    a.0[0][2] = -42.0;
    a.0[1][2] = 123.0;
    let (l_pert, d_pert) = ldl(&a).unwrap();
    assert_eq!(l_clean, l_pert);
    assert_eq!(d_clean, d_pert);
}

#[test]
fn ldl_reproduces_spd_1x1_and_2x2() {
    let a1 = Mat::<1, 1>([[4.0]]);
    let (l1, d1) = ldl(&a1).unwrap();
    assert_eq!(l1.0[0][0], 1.0);
    assert_eq!(d1[0], 4.0);

    let a2 = Mat::<2, 2>([[4.0, 1.0], [1.0, 3.0]]);
    let (l2, d2) = ldl(&a2).unwrap();
    assert!(max_diff(&ldl_reconstruct(&l2, &d2), &a2) < 1e-5);
}

#[test]
fn ldl_rejects_zero_matrix() {
    use stm32_navcore::linalg::LinalgError;
    let z = Mat::<3, 3>::zeros();
    assert_eq!(ldl(&z), Err(LinalgError::NotPositiveDefinite));
}

#[test]
fn ldl_solve_rejects_invalid_inputs() {
    use stm32_navcore::linalg::LinalgError;
    let a = spd::<3>();
    let (l, d) = ldl(&a).unwrap();
    let b_nan = [f32::NAN, 1.0, 2.0];
    assert_eq!(ldl_solve(&l, &d, &b_nan), Err(LinalgError::NonFinite));
    let d_zero = [1.0, 0.0, 1.0];
    let b = [1.0, 2.0, 3.0];
    assert_eq!(ldl_solve(&l, &d_zero, &b), Err(LinalgError::ZeroPivot));
}
