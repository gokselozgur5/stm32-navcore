use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::slerp::slerp;

#[test]
fn slerp_normalizes_inputs_before_interpolating() {
    let h = core::f32::consts::FRAC_1_SQRT_2;
    let a = Quaternion::new(2.0, 0.0, 0.0, 0.0);
    let b = Quaternion::new(h, 0.0, 0.0, h);
    let mid = slerp(&a, &b, 0.5).unwrap();
    let angle_deg = (2.0 * libm::atan2f(mid.z, mid.w)).to_degrees();
    assert!(
        (angle_deg - 45.0).abs() < 0.1,
        "midpoint is {angle_deg} deg"
    );
    let start = slerp(&a, &b, 0.0).unwrap();
    assert!(
        (start.norm() - 1.0).abs() < 1e-6,
        "t = 0 has norm {}",
        start.norm()
    );
}
