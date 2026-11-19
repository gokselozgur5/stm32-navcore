use stm32_navcore::linalg::jacobi_eig::jacobi_eig;
use stm32_navcore::math::Mat;

#[test]
fn small_magnitude_symmetric_matrix_is_diagonalized() {
    let a = Mat([[1.0e-6f32, 5.0e-7], [5.0e-7, 1.0e-6]]);
    let eig = jacobi_eig(&a).unwrap();
    let lo = eig.values[0].min(eig.values[1]);
    let hi = eig.values[0].max(eig.values[1]);
    assert!((lo - 5.0e-7).abs() < 1.0e-9, "smallest eigenvalue = {lo:e}");
    assert!((hi - 1.5e-6).abs() < 1.0e-9, "largest eigenvalue = {hi:e}");
}
