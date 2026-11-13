use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::attitude_error::attitude_error;
use stm32_navcore::rotation::so3_exp::so3_exp;

fn base() -> Quaternion {
    Quaternion::new(0.7, 0.2, -0.5, 0.4).normalize().unwrap()
}

#[test]
fn recovers_applied_rotation() {
    let delta = so3_exp([0.3, -0.2, 0.5]);
    let e = attitude_error(&base(), &base().mul(&delta)).unwrap();
    let want = (0.09f32 + 0.04 + 0.25).sqrt();
    assert!((e.angle - want).abs() < 1e-5);
    assert!((e.axis[0] - 0.3 / want).abs() < 1e-5);
    assert!((e.axis[2] - 0.5 / want).abs() < 1e-5);
}
