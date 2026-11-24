use stm32_navcore::math::linalg::ud::{ud_factor, ud_reconstruct};
use stm32_navcore::math::Mat;
use stm32_navcore::nav::ekf::NavEkf;

fn max_diff(a: &Mat<8, 8>, b: &Mat<8, 8>) -> f32 {
    a.sub(b).0.iter().flatten().fold(0.0, |m, v| m.max(v.abs()))
}

#[test]
fn nav_ekf_covariance_round_trips() {
    let p = NavEkf::new().cov;
    let (u, d) = ud_factor(&p).unwrap();
    assert!(max_diff(&ud_reconstruct(&u, &d), &p) < 1e-5);
}

#[test]
fn correlated_covariance_round_trips() {
    let mut p = NavEkf::new().cov;
    p.0[0][3] = 3.0;
    p.0[3][0] = 3.0;
    p.0[1][7] = -0.5;
    p.0[7][1] = -0.5;
    let (u, d) = ud_factor(&p).unwrap();
    assert!(max_diff(&ud_reconstruct(&u, &d), &p) < 1e-4);
}

#[test]
fn non_positive_pivot_is_reported() {
    use stm32_navcore::math::linalg::LinalgError;
    let mut p = NavEkf::new().cov;
    p.0[4][4] = -1.0;
    assert_eq!(ud_factor(&p), Err(LinalgError::NotPositiveDefinite));
}

#[test]
fn ud_reproduces_spd_1x1_and_2x2() {
    let p1 = Mat::<1, 1>([[4.0]]);
    let (u1, d1) = ud_factor(&p1).unwrap();
    assert!((ud_reconstruct(&u1, &d1).0[0][0] - 4.0).abs() < 1e-6);

    let p2 = Mat::<2, 2>([[4.0, 1.0], [1.0, 2.0]]);
    let (u2, d2) = ud_factor(&p2).unwrap();
    let r2 = ud_reconstruct(&u2, &d2);
    assert!((r2.0[0][0] - 4.0).abs() < 1e-5);
    assert!((r2.0[0][1] - 1.0).abs() < 1e-5);
    assert!((r2.0[1][1] - 2.0).abs() < 1e-5);
}

#[test]
fn ud_det_reproduces_determinant() {
    use stm32_navcore::math::linalg::ud::ud_det;
    let p = Mat::<2, 2>([[4.0, 1.0], [1.0, 2.0]]);
    let (_u, d) = ud_factor(&p).unwrap();
    let det = ud_det(&d);
    assert!((det - 7.0).abs() < 1e-5);
}

#[test]
fn ud_log_det_matches_log_of_det() {
    use stm32_navcore::math::linalg::ud::{ud_det, ud_log_det};
    let p = Mat::<2, 2>([[4.0, 1.0], [1.0, 2.0]]);
    let (_u, d) = ud_factor(&p).unwrap();
    let log_det = ud_log_det(&d);
    let expected = libm::logf(ud_det(&d));
    assert!((log_det - expected).abs() < 1e-4);
}

#[test]
fn rejects_zero_matrix() {
    use stm32_navcore::math::linalg::LinalgError;
    let z = Mat::<3, 3>::zeros();
    assert_eq!(ud_factor(&z), Err(LinalgError::NotPositiveDefinite));
}

#[test]
fn ud_reads_only_upper_triangle() {
    let mut p = Mat::<2, 2>([[4.0, 1.0], [1.0, 2.0]]);
    let clean = ud_factor(&p).unwrap();
    p.0[1][0] = 999.0;
    let perturbed = ud_factor(&p).unwrap();
    assert_eq!(clean, perturbed);
}

#[test]
fn rejects_non_finite_entries() {
    use stm32_navcore::math::linalg::LinalgError;
    let mut p = Mat::<2, 2>([[4.0, 1.0], [1.0, 2.0]]);
    p.0[0][1] = f32::NAN;
    assert_eq!(ud_factor(&p), Err(LinalgError::NonFinite));
    p.0[0][1] = f32::INFINITY;
    assert_eq!(ud_factor(&p), Err(LinalgError::NonFinite));
}
