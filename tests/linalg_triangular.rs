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
