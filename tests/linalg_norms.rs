use stm32_navcore::linalg::norms::{
    all_finite, frobenius_norm, infinity_norm, is_finite, max_abs_norm, norm_inf, trace,
};
use stm32_navcore::math::Mat;

#[test]
fn norm_inf_matches_infinity_norm() {
    let a = Mat::<2, 2>([[1.0, -4.0], [-2.0, 3.0]]);
    assert!((norm_inf(&a) - 5.0).abs() < 1e-6);
}

#[test]
fn infinity_norm_matches_known_values() {
    let a = Mat::<2, 3>([[-3.0, 5.0, 1.0], [2.0, -8.0, 1.0]]);
    assert!((infinity_norm(&a) - 11.0).abs() < 1e-6);

    let b = Mat::<3, 2>([[2.5, -1.0], [-4.0, -3.5], [0.0, 7.0]]);
    assert!((infinity_norm(&b) - 7.5).abs() < 1e-6);
}

#[test]
fn frobenius_norm_matches_known_values() {
    let a = Mat::<2, 2>([[3.0, 4.0], [0.0, 0.0]]);
    assert!((frobenius_norm(&a) - 5.0).abs() < 1e-6);

    let b = Mat::<2, 3>([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    let expected = libm::sqrtf(91.0);
    assert!((frobenius_norm(&b) - expected).abs() < 1e-6);
}

#[test]
fn max_abs_norm_matches_known_values() {
    let a = Mat::<3, 2>([[1.0, -7.5], [3.0, 4.2], [-9.1, 0.0]]);
    assert!((max_abs_norm(&a) - 9.1).abs() < 1e-6);
}

#[test]
fn trace_matches_known_values() {
    let eye = Mat::<4, 4>::identity();
    assert!((trace(&eye) - 4.0).abs() < 1e-6);

    let m = Mat::<3, 3>([[1.5, 2.0, 3.0], [4.0, -2.5, 6.0], [7.0, 8.0, 10.0]]);
    assert!((trace(&m) - 9.0).abs() < 1e-6);
}

#[test]
fn all_finite_detects_nan_and_inf() {
    let mut m = Mat::<2, 2>([[1.0, 2.0], [3.0, 4.0]]);
    assert!(all_finite(&m));

    m.0[0][1] = f32::NAN;
    assert!(!all_finite(&m));

    m.0[0][1] = f32::INFINITY;
    assert!(!all_finite(&m));

    m.0[0][1] = f32::NEG_INFINITY;
    assert!(!all_finite(&m));
}

#[test]
fn is_finite_detects_nan_and_inf() {
    let mut m = Mat::<2, 2>([[1.0, 2.0], [3.0, 4.0]]);
    assert!(is_finite(&m));

    m.0[1][0] = f32::NAN;
    assert!(!is_finite(&m));

    m.0[1][0] = f32::INFINITY;
    assert!(!is_finite(&m));
}
