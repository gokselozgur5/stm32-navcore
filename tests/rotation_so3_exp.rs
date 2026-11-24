use stm32_navcore::math::rotation::so3_exp::{so3_exp, so3_exp_dcm, so3_log, so3_log_dcm};
use stm32_navcore::math::rotation::{Dcm, RotationError};
use stm32_navcore::math::{Mat, Quaternion};

const AXES: [[f32; 3]; 4] = [
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.6, 0.0, 0.8],
    [0.48, -0.64, 0.6],
];

fn worst_error(angle: f32) -> f32 {
    let mut worst = 0.0f32;
    for a in AXES {
        let v = [a[0] * angle, a[1] * angle, a[2] * angle];
        let back = so3_log(&so3_exp(v)).unwrap();
        for i in 0..3 {
            worst = worst.max((back[i] - v[i]).abs());
        }
    }
    worst
}

fn worst_dcm_error(angle: f32) -> f32 {
    let mut worst = 0.0f32;
    for a in AXES {
        let v = [a[0] * angle, a[1] * angle, a[2] * angle];
        let back = so3_log_dcm(&so3_exp_dcm(v)).unwrap();
        for i in 0..3 {
            worst = worst.max((back[i] - v[i]).abs());
        }
    }
    worst
}

#[test]
fn zero_vector_maps_to_identity() {
    assert_eq!(so3_exp([0.0; 3]), Quaternion::IDENTITY);
    assert_eq!(so3_log(&Quaternion::IDENTITY).unwrap(), [0.0; 3]);
}

#[test]
fn log_inverts_exp_across_angles() {
    let limit = core::f32::consts::PI - 1e-3;
    let mut angle = 1e-8f32;
    while angle < limit {
        let e = worst_error(angle);
        assert!(e < 1e-6, "angle {angle} error {e}");
        angle *= 1.5;
    }
    assert!(worst_error(limit) < 1e-6);
}

#[test]
fn exp_gives_unit_quaternion() {
    for angle in [1e-8f32, 1e-3, 0.5, 3.0] {
        let q = so3_exp([0.0, angle, 0.0]);
        assert!((q.norm() - 1.0).abs() < 1e-6);
    }
}

#[test]
fn exp_dcm_matches_known_rotation() {
    let half_pi = core::f32::consts::FRAC_PI_2;
    let r = so3_exp_dcm([0.0, 0.0, half_pi]);
    assert!((r.0 .0[0][1] - (-1.0)).abs() < 1e-6);
    assert!((r.0 .0[1][0] - 1.0).abs() < 1e-6);
}

#[test]
fn log_dcm_recovers_known_rotation() {
    let half_pi = core::f32::consts::FRAC_PI_2;
    let r = so3_exp_dcm([0.0, 0.0, half_pi]);
    let v = so3_log_dcm(&r).unwrap();
    assert!((v[0] - 0.0).abs() < 1e-6);
    assert!((v[1] - 0.0).abs() < 1e-6);
    assert!((v[2] - half_pi).abs() < 1e-6);
}

#[test]
fn log_dcm_inverts_exp_dcm_across_angles() {
    let limit = core::f32::consts::PI - 1e-3;
    let mut angle = 1e-8f32;
    while angle < limit {
        let e = worst_dcm_error(angle);
        assert!(e < 1e-5, "dcm angle {angle} error {e}");
        angle *= 1.5;
    }
}

#[test]
fn log_dcm_rejects_non_orthonormal() {
    let mut bad = Mat::<3, 3>::identity();
    bad.0[0][0] = 2.0;
    assert_eq!(so3_log_dcm(&Dcm(bad)), Err(RotationError::NotOrthonormal));
}
