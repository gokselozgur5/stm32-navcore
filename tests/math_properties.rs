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

#[test]
fn transpose_identities_hold() {
    let mut s = 7;
    for _ in 0..200 {
        let a: Mat<3, 4> = rand_mat(&mut s);
        let b: Mat<4, 2> = rand_mat(&mut s);
        assert_eq!(a.transpose().transpose(), a);
        let lhs = a.matmul(&b).transpose();
        let rhs = b.transpose().matmul(&a.transpose());
        assert!(close(&lhs, &rhs, 1e-6));
    }
}

#[test]
fn inverse_times_matrix_is_identity() {
    let mut s = 99;
    let mut checked = 0;
    for _ in 0..500 {
        let a: Mat<3, 3> = rand_mat(&mut s);
        let b: Mat<2, 2> = rand_mat(&mut s);
        if let Some(inv) = a.inv_3x3() {
            assert!(close(&a.matmul(&inv), &Mat::identity(), 1e-2));
            checked += 1;
        }
        if let Some(inv) = b.inv_2x2() {
            assert!(close(&b.matmul(&inv), &Mat::identity(), 1e-2));
        }
    }
    assert!(checked > 400);
}
