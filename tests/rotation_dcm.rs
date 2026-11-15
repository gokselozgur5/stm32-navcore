use stm32_navcore::math::Mat;
use stm32_navcore::rotation::{Dcm, RotationError};

fn rz(a: f32) -> Dcm {
    let (s, c) = (a.sin(), a.cos());
    Dcm(Mat([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]))
}

#[test]
fn try_new_accepts_rotations() {
    assert!(Dcm::try_new(Mat::identity()).is_ok());
    assert!(Dcm::try_new(rz(0.7).0).is_ok());
}

#[test]
fn try_new_rejects_scaled_reflected_and_nan() {
    let mut scaled = rz(0.3).0;
    scaled.0[0][0] *= 1.01;
    assert_eq!(Dcm::try_new(scaled), Err(RotationError::NotOrthonormal));
    let mut flip = Mat::<3, 3>::identity();
    flip.0[2][2] = -1.0;
    assert!(Dcm::try_new(flip).is_err());
    let mut bad = Mat::<3, 3>::identity();
    bad.0[1][1] = f32::NAN;
    assert!(Dcm::try_new(bad).is_err());
}

#[test]
fn multiply_composes_angles() {
    let p = rz(0.4).mul(&rz(0.5));
    let want = rz(0.9);
    for (row_p, row_want) in p.0 .0.iter().zip(want.0 .0.iter()) {
        for (&val_p, &val_want) in row_p.iter().zip(row_want.iter()) {
            assert!((val_p - val_want).abs() < 1e-6);
        }
    }
}

#[test]
fn transpose_is_inverse() {
    let r = rz(1.1);
    let p = r.transpose().mul(&r);
    assert!(p.orthogonality_error() < 1e-6);
    assert!((p.0 .0[0][0] - 1.0).abs() < 1e-6);
}

#[test]
fn is_orthonormal_validates_so3() {
    assert!(Dcm::identity().is_orthonormal());
    assert!(rz(0.5).is_orthonormal());
    let mut bad = Mat::<3, 3>::identity();
    bad.0[0][0] = 1.05;
    assert!(!Dcm(bad).is_orthonormal());
}

#[test]
fn rotate_vec_and_transpose_rotate_vectors() {
    let r = rz(core::f32::consts::FRAC_PI_2);
    let v = [1.0, 0.0, 0.0];
    let v_rot = r.rotate_vec(v);
    assert!(v_rot[0].abs() < 1e-6);
    assert!((v_rot[1] - 1.0).abs() < 1e-6);
    let v_back = r.rotate_transposed(v_rot);
    assert!((v_back[0] - 1.0).abs() < 1e-6);
    assert!(v_back[1].abs() < 1e-6);
}

#[test]
fn elementary_axis_rotations_are_orthonormal() {
    let rx = Dcm::from_rot_x(0.3);
    let ry = Dcm::from_rot_y(0.4);
    let rz_rot = Dcm::from_rot_z(0.5);
    assert!(rx.is_orthonormal());
    assert!(ry.is_orthonormal());
    assert!(rz_rot.is_orthonormal());
    let v = [0.0, 1.0, 0.0];
    let v_rot = rx.rotate_vec(v);
    assert!((v_rot[1] - 0.3f32.cos()).abs() < 1e-6);
    assert!((v_rot[2] - 0.3f32.sin()).abs() < 1e-6);
}

#[test]
fn from_euler_zyx_composes_intrinsic_rotations() {
    let dcm = Dcm::from_euler_zyx(0.1, 0.2, 0.3);
    assert!(dcm.is_orthonormal());
    let expected = Dcm::from_rot_z(0.3)
        .mul(&Dcm::from_rot_y(0.2))
        .mul(&Dcm::from_rot_x(0.1));
    for (row_a, row_b) in dcm.0 .0.iter().zip(expected.0 .0.iter()) {
        for (&val_a, &val_b) in row_a.iter().zip(row_b.iter()) {
            assert!((val_a - val_b).abs() < 1e-6);
        }
    }
}

#[test]
fn ops_mul_composes_dcm_and_rotates_vector() {
    let r1 = Dcm::from_rot_z(0.2);
    let r2 = Dcm::from_rot_z(0.3);
    let r12 = r1 * r2;
    assert!((r12.0 .0[0][0] - 0.5f32.cos()).abs() < 1e-6);
    let v = [1.0, 0.0, 0.0];
    let v_rot = r12 * v;
    assert!((v_rot[0] - 0.5f32.cos()).abs() < 1e-6);
    assert!((v_rot[1] - 0.5f32.sin()).abs() < 1e-6);
}
