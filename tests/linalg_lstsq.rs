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
