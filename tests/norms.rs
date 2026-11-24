use stm32_navcore::math::linalg::norms::{infinity_norm, max_abs_norm, norm_inf};
use stm32_navcore::math::Mat;

#[test]
fn infinity_norm_propagates_nan_row_sum() {
    let a = Mat::<2, 2>([[f32::NAN, 1.0], [2.0, 3.0]]);
    assert!(infinity_norm(&a).is_nan());
    assert!(norm_inf(&a).is_nan());
}

#[test]
fn max_abs_norm_propagates_nan_entry() {
    let a = Mat::<2, 2>([[f32::NAN, 1.0], [0.0, 0.0]]);
    assert!(max_abs_norm(&a).is_nan());
}
