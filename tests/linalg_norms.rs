use stm32_navcore::linalg::norms::{all_finite, frobenius_norm, max_abs_norm, trace};
use stm32_navcore::math::Mat;

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
