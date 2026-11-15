use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::attitude_error::{attitude_error, attitude_error_vector};
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

#[test]
fn invariant_to_quaternion_sign() {
    let b = base().mul(&so3_exp([0.3, -0.2, 0.5]));
    let nb = Quaternion::new(-b.w, -b.x, -b.y, -b.z);
    let e1 = attitude_error(&base(), &b).unwrap();
    let e2 = attitude_error(&base(), &nb).unwrap();
    assert!((e1.angle - e2.angle).abs() < 1e-6);
    assert!((e1.axis[1] - e2.axis[1]).abs() < 1e-6);
}

#[test]
fn identical_attitudes_have_zero_error() {
    let e = attitude_error(&base(), &base()).unwrap();
    assert!(e.angle < 1e-3);
}

#[test]
fn rotation_vector_and_degree_methods() {
    let delta = so3_exp([0.1, 0.0, 0.0]);
    let e = attitude_error(&base(), &base().mul(&delta)).unwrap();
    let rvec = e.rotation_vector();
    assert!((rvec[0] - 0.1).abs() < 1e-4);
    assert!(rvec[1].abs() < 1e-5);
    assert!(rvec[2].abs() < 1e-5);
    let deg = e.angle_deg();
    assert!((deg - 0.1 * (180.0 / core::f32::consts::PI)).abs() < 1e-3);
}

#[test]
fn attitude_error_vector_matches() {
    let delta = so3_exp([0.2, -0.1, 0.4]);
    let target = base().mul(&delta);
    let v = attitude_error_vector(&base(), &target).unwrap();
    let e = attitude_error(&base(), &target).unwrap();
    let expected = e.rotation_vector();
    for i in 0..3 {
        assert!((v[i] - expected[i]).abs() < 1e-6);
    }
}

#[test]
fn handles_180_degree_rotation() {
    let q_180 = Quaternion::new(0.0, 0.0, 0.0, 1.0);
    let e = attitude_error(&Quaternion::IDENTITY, &q_180).unwrap();
    assert!((e.angle - core::f32::consts::PI).abs() < 1e-5);
    assert!((e.axis[2].abs() - 1.0).abs() < 1e-5);
}
