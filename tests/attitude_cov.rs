use stm32_navcore::attitude::cov::propagate_attitude_cov;
use stm32_navcore::attitude::AttitudeError;
use stm32_navcore::math::Mat;

#[test]
fn test_repro_attitude_cov_propagation() {
    let cov = Mat::<3, 3>::identity();
    let q_cov = propagate_attitude_cov(&cov, 0.01, 0.1).unwrap();
    assert!(q_cov.0[0][0] > 1.0);
}

#[test]
fn test_attitude_cov_symmetric_and_pd() {
    let cov = Mat::<3, 3>::identity();
    let res = propagate_attitude_cov(&cov, 0.05, 0.1).unwrap();

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
    let cov = Mat::<3, 3>::identity();
    let res1 = propagate_attitude_cov(&cov, 0.01, 0.1).unwrap();
    let res2 = propagate_attitude_cov(&cov, 0.05, 0.1).unwrap();
    let res3 = propagate_attitude_cov(&cov, 0.05, 0.5).unwrap();

    assert!(res2.0[0][0] > res1.0[0][0]);
    assert!(res3.0[0][0] > res2.0[0][0]);
}

#[test]
fn test_attitude_cov_invalid_inputs() {
    let cov = Mat::<3, 3>::identity();

    assert_eq!(
        propagate_attitude_cov(&cov, -0.1, 0.1),
        Err(AttitudeError::InvalidInput)
    );
    assert_eq!(
        propagate_attitude_cov(&cov, 0.1, 0.0),
        Err(AttitudeError::InvalidInput)
    );

    let nan_cov = Mat::<3, 3>([[f32::NAN; 3]; 3]);
    assert_eq!(
        propagate_attitude_cov(&nan_cov, 0.1, 0.1),
        Err(AttitudeError::NonFinite)
    );
}

#[test]
fn test_repro_issue_338_anisotropic_covariance_propagation() {
    let cov = Mat::<3, 3>([[0.01, 0.002, 0.0], [0.002, 0.04, 0.0], [0.0, 0.0, 0.09]]);
    let gyro_var = 0.05;
    let dt = 0.2;
    let expected_noise = gyro_var * dt;

    let res = propagate_attitude_cov(&cov, gyro_var, dt).unwrap();

    assert!((res.0[0][0] - (0.01 + expected_noise)).abs() < 1e-6);
    assert!((res.0[1][1] - (0.04 + expected_noise)).abs() < 1e-6);
    assert!((res.0[2][2] - (0.09 + expected_noise)).abs() < 1e-6);
    assert!((res.0[0][1] - 0.002).abs() < 1e-6);
    assert!((res.0[1][0] - 0.002).abs() < 1e-6);
    assert_eq!(res.0[0][2], 0.0);
    assert_eq!(res.0[2][0], 0.0);
    assert_eq!(res.0[1][2], 0.0);
    assert_eq!(res.0[2][1], 0.0);
}
