use stm32_navcore::mahony_ahrs::{
    mahony_ahrs_6dof_step, mahony_ahrs_9dof_step, MahonyAhrs, MahonyAhrsError, MahonyInputs,
};
use stm32_navcore::math::Quaternion;

#[test]
fn test_mahony_initialization_and_reset() {
    let mut ahrs = MahonyAhrs::default();
    assert_eq!(ahrs.quaternion(), Quaternion::IDENTITY);
    assert_eq!(ahrs.gyro_bias(), [0.0; 3]);
    assert!((ahrs.kp - MahonyAhrs::DEFAULT_KP).abs() < 1e-6);

    let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let with_q = MahonyAhrs::with_attitude(2.0, 0.1, q).unwrap();
    assert_eq!(with_q.quaternion(), q);

    ahrs.set_gyro_bias([0.01, -0.02, 0.03]);
    assert_eq!(ahrs.gyro_bias(), [0.01, -0.02, 0.03]);
    ahrs.reset_gyro_bias();
    assert_eq!(ahrs.gyro_bias(), [0.0; 3]);
}

#[test]
fn test_mahony_invalid_dt_rejected() {
    let mut ahrs = MahonyAhrs::default();
    let g_ok = [0.0, 0.0, 0.0];
    let a_ok = [0.0, 0.0, 9.81];

    assert_eq!(
        ahrs.update(g_ok, a_ok, 0.0),
        Err(MahonyAhrsError::InvalidDt)
    );
    assert_eq!(
        ahrs.update(g_ok, a_ok, -0.01),
        Err(MahonyAhrsError::InvalidDt)
    );
    assert_eq!(
        ahrs.update(g_ok, a_ok, f32::NAN),
        Err(MahonyAhrsError::InvalidDt)
    );
}

#[test]
fn test_mahony_non_finite_inputs_rejected() {
    let mut ahrs = MahonyAhrs::default();
    let g_ok = [0.0, 0.0, 0.0];
    let a_ok = [0.0, 0.0, 9.81];

    let g_nan = [f32::NAN, 0.0, 0.0];
    assert_eq!(
        ahrs.update(g_nan, a_ok, 0.01),
        Err(MahonyAhrsError::NonFinite)
    );

    let a_inf = [0.0, f32::INFINITY, 9.81];
    assert_eq!(
        ahrs.update(g_ok, a_inf, 0.01),
        Err(MahonyAhrsError::NonFinite)
    );
}

#[test]
fn test_mahony_pure_gyro_integration() {
    let mut ahrs = MahonyAhrs::new(0.0, 0.0);
    let dt = 0.01;
    let roll_rate = 0.5;

    for _ in 0..100 {
        ahrs.update([roll_rate, 0.0, 0.0], [0.0, 0.0, 0.0], dt)
            .unwrap();
    }
    let (roll, pitch, yaw) = ahrs.euler_zyx();
    assert!((roll - 0.5).abs() < 0.02);
    assert!(pitch.abs() < 1e-3);
    assert!(yaw.abs() < 1e-3);
}

#[test]
fn test_mahony_gravity_tilt_convergence() {
    let mut ahrs = MahonyAhrs::new(2.0, 0.0);
    let target_pitch = 20.0f32.to_radians();
    let ax = -libm::sinf(target_pitch) * 9.80665;
    let az = libm::cosf(target_pitch) * 9.80665;
    let dt = 0.01;

    for _ in 0..1000 {
        ahrs.update([0.0, 0.0, 0.0], [ax, 0.0, az], dt).unwrap();
    }
    let (roll, pitch, yaw) = ahrs.euler_zyx();
    assert!(roll.abs() < 0.5f32.to_radians());
    assert!((pitch - target_pitch).abs() < 0.5f32.to_radians());
    assert!(yaw.abs() < 0.5f32.to_radians());
}

#[test]
fn test_mahony_gyro_bias_tracking() {
    let mut ahrs = MahonyAhrs::new(2.0, 0.1);
    let injected_bias = [0.02, -0.015, 0.0];
    let static_accel = [0.0, 0.0, 9.81];

    for _ in 0..6000 {
        ahrs.update(injected_bias, static_accel, 0.01).unwrap();
    }
    let est_bias = ahrs.gyro_bias();
    assert!((est_bias[0] - injected_bias[0]).abs() < 0.003);
    assert!((est_bias[1] - injected_bias[1]).abs() < 0.003);
}

#[test]
fn test_mahony_9dof_heading_fusion() {
    let mut ahrs = MahonyAhrs::new(1.0, 0.0);
    let dt = 0.01;
    let heading = 45.0f32.to_radians();
    let mx = libm::cosf(heading);
    let my = -libm::sinf(heading);

    for _ in 0..500 {
        ahrs.update_9dof([0.0; 3], [0.0, 0.0, 9.81], [mx, my, 0.0], dt)
            .unwrap();
    }
    let (_, _, yaw) = ahrs.euler_zyx();
    assert!((yaw - heading).abs() < 1.0f32.to_radians());
}

#[test]
fn test_mahony_step_equivalence_and_norm_stability() {
    let mut ahrs = MahonyAhrs::default();
    let gyro = [0.05, -0.02, 0.01];
    let accel = [0.1, -0.2, 9.80];
    let dt = 0.01;

    for _ in 0..5_000 {
        let (q_step, b_step) = mahony_ahrs_6dof_step(
            ahrs.quaternion(),
            gyro,
            accel,
            ahrs.gyro_bias(),
            ahrs.kp,
            ahrs.ki,
            dt,
        )
        .unwrap();
        let q_up = ahrs.update(gyro, accel, dt).unwrap();
        assert_eq!(q_step, q_up);
        assert_eq!(b_step, ahrs.gyro_bias());
        assert!((q_up.norm() - 1.0).abs() < 1e-5);
    }
}

#[test]
fn test_mahony_9dof_step_equivalence() {
    let mut ahrs = MahonyAhrs::default();
    let g = [0.01, 0.02, 0.03];
    let a = [0.0, 0.0, 9.81];
    let m = [1.0, 0.0, 0.0];
    let (q, b) = mahony_ahrs_9dof_step(
        ahrs.quaternion(),
        MahonyInputs {
            gyro: g,
            accel: a,
            mag: m,
            gyro_bias: ahrs.gyro_bias(),
            kp: ahrs.kp,
            ki: ahrs.ki,
            dt: 0.01,
        },
    )
    .unwrap();
    let q_up = ahrs.update_9dof(g, a, m, 0.01).unwrap();
    assert_eq!(q, q_up);
    assert_eq!(b, ahrs.gyro_bias());
}
