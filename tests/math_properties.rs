use stm32_navcore::math::Mat;

fn next(s: &mut u32) -> f32 {
    *s = s.wrapping_mul(1664525).wrapping_add(1013904223);
    ((*s >> 8) as f32 / 16777216.0) * 2.0 - 1.0
}

fn rand_mat<const R: usize, const C: usize>(s: &mut u32) -> Mat<R, C> {
    let mut m = Mat::<R, C>::zeros();
    for r in 0..R {
        for c in 0..C {
            m.0[r][c] = next(s);
        }
    }
    m
}

fn close<const R: usize, const C: usize>(a: &Mat<R, C>, b: &Mat<R, C>, tol: f32) -> bool {
    (0..R).all(|r| (0..C).all(|c| (a.0[r][c] - b.0[r][c]).abs() <= tol))
}

#[test]
fn matmul_is_associative_within_tolerance() {
    let mut s = 1;
    for _ in 0..200 {
        let a: Mat<3, 4> = rand_mat(&mut s);
        let b: Mat<4, 2> = rand_mat(&mut s);
        let c: Mat<2, 3> = rand_mat(&mut s);
        let left = a.matmul(&b).matmul(&c);
        let right = a.matmul(&b.matmul(&c));
        assert!(close(&left, &right, 1e-5));
    }
}
