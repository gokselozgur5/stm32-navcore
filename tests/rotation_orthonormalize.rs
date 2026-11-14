use stm32_navcore::math::{Mat, Quaternion};
use stm32_navcore::rotation::orthonormalize::skew;
use stm32_navcore::rotation::Dcm;

#[test]
fn skew_matches_cross_product() {
    let (a, b) = ([1.0f32, -2.0, 0.5], [0.3f32, 0.4, -1.0]);
    let got = skew(a).matmul(&Mat([[b[0]], [b[1]], [b[2]]]));
    let want = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    for (i, w) in want.iter().enumerate() {
        assert!((got.0[i][0] - w).abs() < 1e-6);
    }
}

fn noisy() -> Dcm {
    let q = Quaternion::new(0.8, 0.3, -0.4, 0.3);
    let mut d = Dcm::from_quaternion(&q).unwrap();
    let mut s: u32 = 7;
    for row in d.0 .0.iter_mut() {
        for x in row.iter_mut() {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            *x += ((s >> 8) as f32 / 8388608.0 - 1.0) * 1e-3;
        }
    }
    d
}

#[test]
fn noise_is_removed() {
    let d = noisy();
    assert!(d.orthogonality_error() > 1e-4);
    let fixed = d.orthonormalize().unwrap();
    assert!(fixed.orthogonality_error() < 1e-6);
    assert!((fixed.det() - 1.0).abs() < 1e-5);
}

#[test]
fn non_finite_input_is_rejected() {
    let mut d = noisy();
    d.0 .0[0][0] = f32::NAN;
    assert!(d.orthonormalize().is_err());
}

#[test]
fn singular_zero_matrix_is_rejected() {
    let d = Dcm(Mat::<3, 3>::zeros());
    assert!(d.orthonormalize().is_err());
}

#[test]
fn improper_reflection_is_rejected() {
    let mut neg_i = Mat::<3, 3>::identity();
    for i in 0..3 {
        neg_i.0[i][i] = -1.0;
    }
    assert!(Dcm(neg_i).orthonormalize().is_err());
}
