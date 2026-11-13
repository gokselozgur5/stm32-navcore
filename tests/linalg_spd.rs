use stm32_navcore::linalg::spd::{spd_inverse, spd_solve};
use stm32_navcore::math::Mat;

fn spd<const N: usize>() -> Mat<N, N> {
    let mut b = Mat::<N, N>::zeros();
    for (i, row) in b.0.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = ((i * 7 + j * 3) % 5) as f32 * 0.3 - 0.4;
        }
    }
    b.matmul(&b.transpose()).add(&Mat::<N, N>::identity())
}

fn max_diff<const N: usize>(a: &Mat<N, N>, b: &Mat<N, N>) -> f32 {
    a.sub(b).0.iter().flatten().fold(0.0, |m, v| m.max(v.abs()))
}

#[test]
fn inverse_matches_closed_form_3x3() {
    let a = spd::<3>();
    let closed = a.inv_3x3().unwrap();
    assert!(max_diff(&spd_inverse(&a).unwrap(), &closed) < 1e-5);
}
