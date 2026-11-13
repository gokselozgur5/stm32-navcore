use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::slerp::{nlerp, slerp};

fn about_z(angle: f32) -> Quaternion {
    Quaternion::new((0.5 * angle).cos(), 0.0, 0.0, (0.5 * angle).sin())
}

fn angle_of(q: &Quaternion) -> f32 {
    2.0 * q.z.abs().atan2(q.w.abs())
}

#[test]
fn endpoints_are_exact() {
    let (a, b) = (about_z(0.2), about_z(1.4));
    assert_eq!(slerp(&a, &b, 0.0).unwrap(), a);
    assert_eq!(slerp(&a, &b, 1.0).unwrap(), b);
}

#[test]
fn midpoint_is_half_the_angle() {
    let m = slerp(&about_z(0.0), &about_z(1.2), 0.5).unwrap();
    assert!((angle_of(&m) - 0.6).abs() < 1e-5);
}
