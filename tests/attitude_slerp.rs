use stm32_navcore::attitude::slerp::attitude_slerp;
use stm32_navcore::math::Quaternion;

#[test]
fn test_attitude_slerp_endpoints() {
    let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    assert_eq!(attitude_slerp(&q0, &q1, 0.0).unwrap(), q0);
}
