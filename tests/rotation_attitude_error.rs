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
use stm32_navcore::rotation::so3_exp::so3_exp;

#[test]
fn recovers_applied_rotation() {
    let delta = so3_exp([0.3, -0.2, 0.5]);
    let e = attitude_error(&base(), &base().mul(&delta)).unwrap();
    let want = (0.09f32 + 0.04 + 0.25).sqrt();
    assert!((e.angle - want).abs() < 1e-5);
    assert!((e.axis[0] - 0.3 / want).abs() < 1e-5);
    assert!((e.axis[2] - 0.5 / want).abs() < 1e-5);
}
#[test]
fn invariant_to_quaternion_sign() {
    let b = base().mul(&so3_exp([0.3, -0.2, 0.5]));
    let nb = Quaternion::new(-b.w, -b.x, -b.y, -b.z);
    let e1 = attitude_error(&base(), &b).unwrap();
    let e2 = attitude_error(&base(), &nb).unwrap();
    assert!((e1.angle - e2.angle).abs() < 1e-6);
    assert!((e1.axis[1] - e2.axis[1]).abs() < 1e-6);
}
