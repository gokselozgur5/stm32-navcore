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
