use stm32_navcore::math::linalg::cond::{
    cond_est_spd, is_invertible_spd, is_well_conditioned_spd, norm_1, rcond_est_spd,
};
use stm32_navcore::math::linalg::LinalgError;
use stm32_navcore::math::Mat;

#[test]
fn norm_1_matches_hand_computed() {
    let mut m = Mat::<2, 3>::zeros();
    m.0[0] = [1.0, -5.0, 2.0];
    m.0[1] = [-3.0, 2.0, -4.0];
    assert_eq!(norm_1(&m), 7.0);
}

#[test]
fn cond_est_diagonal_matches_exact() {
    let mut d = Mat::<4, 4>::zeros();
    d.0[0][0] = 1.0;
    d.0[1][1] = 2.0;
    d.0[2][2] = 5.0;
    d.0[3][3] = 10.0;
    let est = cond_est_spd(&d).unwrap();
    let exact = 10.0f32;
    assert!((est - exact).abs() < 1e-4);
}

#[test]
fn cond_est_hilbert_within_factor_of_three() {
    let mut h = Mat::<4, 4>::zeros();
    for i in 0..4 {
        for j in 0..4 {
            h.0[i][j] = 1.0 / ((i + j + 1) as f32);
        }
    }
    let est = cond_est_spd(&h).unwrap();
    let exact = 28375.0f32;
    assert!(est > exact / 3.0 && est < exact * 3.0);
}

#[test]
fn cond_est_rejects_non_pd_and_non_finite() {
    let mut non_pd = Mat::<3, 3>::identity();
    non_pd.0[0][0] = -1.0;
    assert_eq!(cond_est_spd(&non_pd), Err(LinalgError::NotPositiveDefinite));

    let mut nan_mat = Mat::<3, 3>::identity();
    nan_mat.0[0][0] = f32::NAN;
    assert_eq!(cond_est_spd(&nan_mat), Err(LinalgError::NonFinite));
}

#[test]
fn cond_est_1x1_and_2x2_exact() {
    let m1 = Mat::<1, 1>([[5.0]]);
    assert!((cond_est_spd(&m1).unwrap() - 1.0).abs() < 1e-4);

    let mut m2 = Mat::<2, 2>::zeros();
    m2.0[0][0] = 2.0;
    m2.0[1][1] = 8.0;
    assert!((cond_est_spd(&m2).unwrap() - 4.0).abs() < 1e-4);
}

#[test]
fn rcond_est_spd_matches_reciprocal() {
    let mut d = Mat::<4, 4>::zeros();
    d.0[0][0] = 1.0;
    d.0[1][1] = 2.0;
    d.0[2][2] = 5.0;
    d.0[3][3] = 10.0;
    let r = rcond_est_spd(&d).unwrap();
    assert!((r - 0.1).abs() < 1e-4);
}

#[test]
fn is_well_conditioned_spd_checks_threshold() {
    let eye = Mat::<3, 3>::identity();
    assert!(is_well_conditioned_spd(&eye, 2.0));
    assert!(!is_well_conditioned_spd(&eye, 0.5));

    let mut non_pd = Mat::<3, 3>::identity();
    non_pd.0[0][0] = -1.0;
    assert!(!is_well_conditioned_spd(&non_pd, 100.0));
}

#[test]
fn is_invertible_spd_checks_tolerance() {
    let eye = Mat::<3, 3>::identity();
    assert!(is_invertible_spd(&eye, 1e-4));

    let mut ill = Mat::<4, 4>::zeros();
    for i in 0..4 {
        for j in 0..4 {
            ill.0[i][j] = 1.0 / ((i + j + 1) as f32);
        }
    }
    assert!(!is_invertible_spd(&ill, 1e-3));
    assert!(is_invertible_spd(&ill, 1e-6));
}
