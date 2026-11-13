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
