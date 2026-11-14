use std::f32;

use stm32_navcore::math::{quaternion, Vector3};
use stm32_navcore::rotation::attitude_error::attitude_error_angle;

#[test]
fn test_attitude_error_angle() {
    let q1 = quaternion::Quaternion::new(0.7071067811865475, 0.7071067811865475, 0.0, 0.0);
    let q2 = quaternion::Quaternion::new(0.7071067811865475, -0.7071067811865475, 0.0, 0.0);

    let expected_angle = f32::consts::PI / 2.0;
    let angle = attitude_error_angle(&q1, &q2);

    assert!((angle - expected_angle).abs() < f32::EPSILON);
}
