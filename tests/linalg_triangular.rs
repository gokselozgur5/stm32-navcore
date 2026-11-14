use stm32_navcore::linalg::triangular::{solve_lower, solve_upper};
use stm32_navcore::math::Mat;

const X: [f32; 4] = [1.0, -2.0, 0.5, 3.0];

fn tri(lower: bool) -> Mat<4, 4> {
    let mut m = Mat::<4, 4>::zeros();
    for (i, row) in m.0.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            if (lower && j <= i) || (!lower && j >= i) {
                *v = 0.5 * (i + j) as f32 - 1.0 + if i == j { 3.0 } else { 0.0 };
            }
        }
    }
    m
}

fn apply(m: &Mat<4, 4>) -> [f32; 4] {
    core::array::from_fn(|i| (0..4).map(|j| m.0[i][j] * X[j]).sum())
}

#[test]
fn lower_recovers_known_x() {
    let m = tri(true);
    let x = solve_lower(&m, &apply(&m)).unwrap();
    assert!(x.iter().zip(X).all(|(a, b)| (a - b).abs() < 1e-5));
}

#[test]
fn upper_recovers_known_x() {
    let m = tri(false);
    let x = solve_upper(&m, &apply(&m)).unwrap();
    assert!(x.iter().zip(X).all(|(a, b)| (a - b).abs() < 1e-5));
}

#[test]
fn zero_pivot_is_an_error() {
    use stm32_navcore::linalg::LinalgError;
    let mut l = tri(true);
    l.0[2][2] = 0.0;
    assert_eq!(solve_lower(&l, &[1.0; 4]), Err(LinalgError::ZeroPivot));
    let mut u = tri(false);
    u.0[1][1] = 0.0;
    assert_eq!(solve_upper(&u, &[1.0; 4]), Err(LinalgError::ZeroPivot));
}

#[test]
fn lower_rejects_non_finite_inputs() {
    use stm32_navcore::linalg::LinalgError;
    let mut m = tri(true);
    m.0[0][0] = f32::NAN;
    assert_eq!(solve_lower(&m, &[1.0; 4]), Err(LinalgError::NonFinite));
    let m = tri(true);
    assert_eq!(
        solve_lower(&m, &[f32::NAN, 1.0, 1.0, 1.0]),
        Err(LinalgError::NonFinite)
    );
}

#[test]
fn upper_rejects_non_finite_inputs() {
    use stm32_navcore::linalg::LinalgError;
    let mut m = tri(false);
    m.0[0][0] = f32::INFINITY;
    assert_eq!(solve_upper(&m, &[1.0; 4]), Err(LinalgError::NonFinite));
    let m = tri(false);
    assert_eq!(
        solve_upper(&m, &[1.0, f32::NAN, 1.0, 1.0]),
        Err(LinalgError::NonFinite)
    );
}

#[test]
fn triangular_solves_small_dimensions() {
    let l1 = Mat([[3.0f32]]);
    assert_eq!(solve_lower(&l1, &[6.0]), Ok([2.0]));
    let u1 = Mat([[4.0f32]]);
    assert_eq!(solve_upper(&u1, &[12.0]), Ok([3.0]));
    let l2 = Mat([[2.0, 0.0], [1.0, 3.0]]);
    assert_eq!(solve_lower(&l2, &[4.0, 8.0]), Ok([2.0, 2.0]));
    let u2 = Mat([[2.0, 1.0], [0.0, 3.0]]);
    assert_eq!(solve_upper(&u2, &[5.0, 6.0]), Ok([1.5, 2.0]));
}
