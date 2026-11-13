use stm32_navcore::linalg::lstsq::lstsq;
use stm32_navcore::math::Mat;

#[test]
fn lstsq_fits_10x3_line_plus_offset() {
    let mut a = Mat::<10, 3>::zeros();
    let mut b = [0.0f32; 10];
    let truth = [2.5f32, -1.2f32, 0.4f32];
    for i in 0..10 {
        let t = i as f32 * 0.5;
        a.0[i][0] = 1.0;
        a.0[i][1] = t;
        a.0[i][2] = t * t;
        b[i] = truth[0] + truth[1] * t + truth[2] * t * t;
    }
    let x = lstsq(&a, &b).unwrap();
    for j in 0..3 {
        assert!((x[j] - truth[j]).abs() < 1e-4);
    }
}

#[test]
fn lstsq_rejects_rank_deficient_and_non_finite() {
    use stm32_navcore::linalg::LinalgError;
    let mut a = Mat::<5, 3>::zeros();
    for i in 0..5 {
        let v = (i + 1) as f32;
        a.0[i][0] = v;
        a.0[i][1] = v * 2.0;
        a.0[i][2] = v * 3.0;
    }
    let b = [1.0f32; 5];
    assert_eq!(lstsq(&a, &b), Err(LinalgError::ZeroPivot));
    let mut bad_b = b;
    bad_b[2] = f32::NAN;
    assert_eq!(lstsq(&a, &bad_b), Err(LinalgError::NonFinite));
    let mut bad_a = a;
    bad_a.0[0][0] = f32::INFINITY;
    assert_eq!(lstsq(&bad_a, &b), Err(LinalgError::NonFinite));
}
