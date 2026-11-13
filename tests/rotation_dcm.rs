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
    for i in 0..3 {
        for j in 0..3 {
            assert!((p.0 .0[i][j] - want.0 .0[i][j]).abs() < 1e-6);
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
