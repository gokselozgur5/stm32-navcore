use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::attitude_error::attitude_error;

fn base() -> Quaternion {
    Quaternion::new(0.7, 0.2, -0.5, 0.4).normalize().unwrap()
}

#[test]
fn identical_attitudes_have_zero_error() {
    let e = attitude_error(&base(), &base()).unwrap();
    assert!(e.angle < 1e-3);
}
