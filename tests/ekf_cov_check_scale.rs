use stm32_navcore::ekf::NavEkf;

fn block(d: f32, upper: f32, lower: f32) -> NavEkf {
    let mut e = NavEkf::new();
    e.cov.0[6][6] = d;
    e.cov.0[7][7] = d;
    e.cov.0[6][7] = upper;
    e.cov.0[7][6] = lower;
    e
}

#[test]
fn tiny_scale_upper_only_correlation_is_rejected() {
    assert!(!block(1e-6, 5e-5, 0.0).covariance_is_positive_definite());
}

#[test]
fn tiny_scale_sign_flipped_correlation_is_rejected() {
    assert!(!block(1e-6, 9e-7, -9e-7).covariance_is_positive_definite());
}

#[test]
fn tiny_scale_singular_block_is_rejected() {
    assert!(!block(1e-6, 1e-6, 1e-6).covariance_is_positive_definite());
}
