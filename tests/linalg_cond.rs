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
