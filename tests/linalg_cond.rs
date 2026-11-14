use stm32_navcore::linalg::cond::{cond_est_spd, norm_1};
use stm32_navcore::linalg::LinalgError;
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
