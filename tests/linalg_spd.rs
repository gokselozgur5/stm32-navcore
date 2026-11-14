use stm32_navcore::linalg::spd::{spd_inverse, spd_solve};
use stm32_navcore::linalg::LinalgError;
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
fn inverse_matches_closed_form_3x3() {
    let a = spd::<3>();
    let closed = a.inv_3x3().unwrap();
    assert!(max_diff(&spd_inverse(&a).unwrap(), &closed) < 1e-5);
}

#[test]
fn solve_matches_closed_form_3x3() {
    let a = spd::<3>();
    let b = [1.0, -2.0, 3.0];
    let x = spd_solve(&a, &b).unwrap();
    let inv = a.inv_3x3().unwrap();
    for (i, xi) in x.iter().enumerate() {
        let e: f32 = (0..3).map(|j| inv.0[i][j] * b[j]).sum();
        assert!((xi - e).abs() < 1e-5);
    }
}

#[test]
fn inverse_6x6_times_input_is_identity() {
    let a = spd::<6>();
    let prod = a.matmul(&spd_inverse(&a).unwrap());
    assert!(max_diff(&prod, &Mat::<6, 6>::identity()) < 1e-4);
}

#[test]
fn solve_rejects_non_finite_vector() {
    let a = spd::<3>();
    assert_eq!(
        spd_solve(&a, &[f32::NAN, 1.0, 1.0]),
        Err(LinalgError::NonFinite)
    );
}

#[test]
fn solve_and_inverse_reject_non_pd() {
    let a = Mat::<3, 3>::zeros();
    let b = [1.0, 2.0, 3.0];
    assert_eq!(spd_solve(&a, &b), Err(LinalgError::NotPositiveDefinite));
    assert_eq!(spd_inverse(&a), Err(LinalgError::NotPositiveDefinite));
}

#[test]
fn solve_and_inverse_1x1_and_2x2() {
    let a1 = Mat::<1, 1>([[4.0]]);
    assert_eq!(spd_solve(&a1, &[12.0]).unwrap(), [3.0]);
    assert_eq!(spd_inverse(&a1).unwrap().0[0][0], 0.25);

    let a2 = Mat::<2, 2>([[2.0, -1.0], [-1.0, 2.0]]);
    let x2 = spd_solve(&a2, &[1.0, 2.0]).unwrap();
    assert!((x2[0] - 1.3333333).abs() < 1e-5);
    assert!((x2[1] - 1.6666666).abs() < 1e-5);
}
