use stm32_navcore::attitude::cov::propagate_attitude_cov;
use stm32_navcore::math::{Mat, Quaternion};

#[test]
fn test_repro_attitude_cov_propagation() {
    let q = Quaternion::IDENTITY;
    let cov = Mat::<3, 3>::identity();
    let q_cov = propagate_attitude_cov(&cov, &q, 0.01, 0.1).unwrap();
    assert!(q_cov.0[0][0] > 1.0);
}

#[test]
fn test_attitude_cov_symmetric_and_pd() {
    let q = Quaternion::IDENTITY;
    let cov = Mat::<3, 3>::identity();
    let res = propagate_attitude_cov(&cov, &q, 0.05, 0.1).unwrap();

    for i in 0..3 {
        for j in 0..3 {
            assert!((res.0[i][j] - res.0[j][i]).abs() < 1e-6);
        }
    }
    for i in 0..3 {
        assert!(res.0[i][i] > 1.0);
    }
}

#[test]
fn test_attitude_cov_monotonic_growth() {
    let q = Quaternion::IDENTITY;
    let cov = Mat::<3, 3>::identity();
    let res1 = propagate_attitude_cov(&cov, &q, 0.01, 0.1).unwrap();
    let res2 = propagate_attitude_cov(&cov, &q, 0.05, 0.1).unwrap();
    let res3 = propagate_attitude_cov(&cov, &q, 0.05, 0.5).unwrap();

    assert!(res2.0[0][0] > res1.0[0][0]);
    assert!(res3.0[0][0] > res2.0[0][0]);
}
