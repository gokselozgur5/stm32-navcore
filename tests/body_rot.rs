use stm32_navcore::body_rot::{BodyRot, BodyRotError};
use stm32_navcore::math::Mat;

#[test]
fn test_identity_rotation_preserves_vectors() {
    let rot = BodyRot::identity();
    assert!(rot.is_orthonormal());

    let v_ned = [1.2, -3.4, 5.6];
    let v_body = rot.rotate_ned_to_body(v_ned).expect("valid rotation");
    assert!((v_body[0] - v_ned[0]).abs() < 1e-6);
    assert!((v_body[1] - v_ned[1]).abs() < 1e-6);
    assert!((v_body[2] - v_ned[2]).abs() < 1e-6);

    let v_roundtrip = rot.rotate_body_to_ned(v_body).expect("valid rotation");
    assert!((v_roundtrip[0] - v_ned[0]).abs() < 1e-6);
    assert!((v_roundtrip[1] - v_ned[1]).abs() < 1e-6);
    assert!((v_roundtrip[2] - v_ned[2]).abs() < 1e-6);
}

#[test]
fn test_euler_roundtrip_recovery() {
    let roll = 0.25f32;
    let pitch = -0.15f32;
    let yaw = 1.10f32;

    let rot = BodyRot::from_euler(roll, pitch, yaw).expect("valid euler");
    assert!(rot.is_orthonormal());

    let (r_rec, p_rec, y_rec) = rot.to_euler().expect("valid euler recovery");
    assert!((r_rec - roll).abs() < 1e-4);
    assert!((p_rec - pitch).abs() < 1e-4);
    assert!((y_rec - yaw).abs() < 1e-4);
}

#[test]
fn test_transform_accel_level_and_gravity() {
    let rot = BodyRot::identity();
    let g = 9.80665f32;
    let f_body = [0.0, 0.0, -g];
    let a_ned = rot.transform_accel_to_ned(f_body, g).expect("valid accel");

    assert!(a_ned[0].abs() < 1e-5);
    assert!(a_ned[1].abs() < 1e-5);
    assert!(a_ned[2].abs() < 1e-5);
}

#[test]
fn test_rejection_of_non_finite_inputs() {
    assert_eq!(
        BodyRot::from_euler(f32::NAN, 0.0, 0.0),
        Err(BodyRotError::NonFinite)
    );
    let rot = BodyRot::identity();
    assert_eq!(
        rot.rotate_ned_to_body([f32::INFINITY, 0.0, 0.0]),
        Err(BodyRotError::NonFinite)
    );
    assert_eq!(
        rot.rotate_body_to_ned([0.0, f32::NAN, 0.0]),
        Err(BodyRotError::NonFinite)
    );
    assert_eq!(
        rot.transform_accel_to_ned([0.0, 0.0, 0.0], f32::NAN),
        Err(BodyRotError::NonFinite)
    );
}

#[test]
fn test_reorthonormalize_recovers_orthogonality() {
    let base = BodyRot::from_euler(0.1, -0.2, 0.3).expect("valid euler");
    let mut drifted = *base.dcm_ned_to_body();
    drifted.0[0][1] += 0.005;
    drifted.0[1][0] += 0.005;

    let unortho = BodyRot { dcm_nb: drifted };
    assert!(!unortho.is_orthonormal());

    let cleaned = unortho.reorthonormalize().expect("reorthonormalize success");
    assert!(cleaned.is_orthonormal());
    assert!((cleaned.det() - 1.0).abs() < 1e-4);
}

#[test]
fn test_dcm_constructors_with_mat_identity() {
    let rot = BodyRot::from_dcm_ned_to_body(Mat::identity()).expect("identity dcm");
    assert!(rot.is_orthonormal());
    let rot_bn = BodyRot::from_dcm_body_to_ned(Mat::identity()).expect("identity dcm");
    assert!(rot_bn.is_orthonormal());
}
