use stm32_navcore::linalg::jacobi_eig::jacobi_eig;
use stm32_navcore::math::Mat;

fn symmetric_matrix<const N: usize>() -> Mat<N, N> {
    let mut b = Mat::<N, N>::zeros();
    for (i, row) in b.0.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = ((i * 7 + j * 3 + 2) % 9) as f32 * 0.25 - 1.0;
        }
    }
    b.add(&b.transpose())
}
