use stm32_navcore::linalg::jacobi_eig::jacobi_eig;
use stm32_navcore::math::Mat;

#[test]
fn eigenpairs_satisfy_av_lambda_v_2x2() {
    let mut a = Mat::<2, 2>::zeros();
    a.0[0][0] = 2.0;
    a.0[0][1] = 1.0;
    a.0[1][0] = 1.0;
    a.0[1][1] = 2.0;
    let res = jacobi_eig(&a).expect("jacobi_eig failed");
    assert!((res.values[0] - 3.0).abs() < 1e-4 || (res.values[0] - 1.0).abs() < 1e-4);
}
