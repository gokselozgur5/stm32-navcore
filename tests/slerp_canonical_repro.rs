use stm32_navcore::attitude::attitude_slerp;
use stm32_navcore::math::rotation::slerp::quat_slerp;
use stm32_navcore::math::rotation::slerp::slerp as rotation_slerp;
use stm32_navcore::math::Quaternion;

#[test]
fn test_three_slerp_paths_produce_identical_results_for_non_unit_quaternions() {
    let a = Quaternion::new(2.0, 0.0, 0.0, 0.0);
    let half_angle = core::f32::consts::FRAC_PI_4; // 45 deg = 90 deg / 2
    let b = Quaternion::new(libm::cosf(half_angle), 0.0, 0.0, libm::sinf(half_angle));
    let t = 0.5f32;

    let res_rotation = rotation_slerp(&a, &b, t).expect("rotation slerp");
    let res_quat = quat_slerp(&a, &b, t).expect("quat_slerp");
    let res_attitude = attitude_slerp(&a, &b, t).expect("attitude_slerp");

    assert_eq!(res_rotation, res_quat);
    assert_eq!(res_rotation, res_attitude);

    // Halfway between 0 and 90 deg is 45 deg, so rotation around z by 45 deg (half angle 22.5 deg)
    let half_res_angle = core::f32::consts::PI / 8.0; // 22.5 deg
    let expected = Quaternion::new(
        libm::cosf(half_res_angle),
        0.0,
        0.0,
        libm::sinf(half_res_angle),
    );

    assert!((res_rotation.w - expected.w).abs() < 1e-6);
    assert!((res_rotation.x - expected.x).abs() < 1e-6);
    assert!((res_rotation.y - expected.y).abs() < 1e-6);
    assert!((res_rotation.z - expected.z).abs() < 1e-6);
}

#[test]
fn test_three_slerp_paths_clamp_out_of_range_t() {
    let a = Quaternion::new(2.0, 0.0, 0.0, 0.0);
    let b = Quaternion::new(0.0, 3.0, 0.0, 0.0);

    let norm_a = a.normalize().unwrap();
    let norm_b = b.normalize().unwrap();

    assert_eq!(rotation_slerp(&a, &b, -0.5).unwrap(), norm_a);
    assert_eq!(quat_slerp(&a, &b, -0.5).unwrap(), norm_a);
    assert_eq!(attitude_slerp(&a, &b, -0.5).unwrap(), norm_a);

    assert_eq!(rotation_slerp(&a, &b, 1.5).unwrap(), norm_b);
    assert_eq!(quat_slerp(&a, &b, 1.5).unwrap(), norm_b);
    assert_eq!(attitude_slerp(&a, &b, 1.5).unwrap(), norm_b);
}
