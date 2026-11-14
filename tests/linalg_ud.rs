use stm32_navcore::ekf::NavEkf;
use stm32_navcore::linalg::ud::{ud_factor, ud_reconstruct};
use stm32_navcore::math::Mat;

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
    let mut p = NavEkf::new().cov;
    p.0[4][4] = -1.0;
    assert!(ud_factor(&p).is_err());
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
