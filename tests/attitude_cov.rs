use stm32_navcore::attitude::cov::propagate_attitude_cov;
use stm32_navcore::math::{Mat, Quaternion};

#[test]
fn test_repro_attitude_cov_propagation() {
    let q = Quaternion::IDENTITY;
    let cov = Mat::<3, 3>::identity();
    let q_cov = propagate_attitude_cov(&cov, &q, 0.01, 0.1).unwrap();
    assert!(q_cov.0[0][0] > 1.0);
}
