use stm32_navcore::math::rotation::so3_exp::{so3_exp_dcm, so3_log_dcm};

#[test]
fn log_dcm_near_pi_retains_accuracy() {
    let angle = core::f32::consts::PI - 0.003;
    let r = so3_exp_dcm([0.0, 0.0, angle]);
    let v = so3_log_dcm(&r).unwrap();
    let norm = libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    assert!(norm <= core::f32::consts::PI);
    assert!(
        (norm - angle).abs() < 1e-4,
        "got norm {norm}, expected {angle}"
    );
}
