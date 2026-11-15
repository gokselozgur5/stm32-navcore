use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::Dcm;

fn max_err(a: &Quaternion, b: &Quaternion) -> f32 {
    let s = if a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0.0 {
        -1.0
    } else {
        1.0
    };
    [a.w - s * b.w, a.x - s * b.x, a.y - s * b.y, a.z - s * b.z]
        .iter()
        .fold(0.0f32, |m, d| m.max(d.abs()))
}

fn roundtrip(q: Quaternion) -> f32 {
    let q = q.normalize().unwrap();
    let d = Dcm::from_quaternion(&q).unwrap();
    assert!(d.orthogonality_error() < 1e-5);
    max_err(&q, &d.to_quaternion())
}

#[test]
fn identity_round_trips() {
    assert!(roundtrip(Quaternion::IDENTITY) < 1e-6);
}

#[test]
fn half_turns_round_trip() {
    for q in [
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ] {
        assert!(roundtrip(Quaternion::new(q.0, q.1, q.2, q.3)) < 1e-6);
    }
    assert!(roundtrip(Quaternion::new(0.0, 1.0, 1.0, 1.0)) < 1e-6);
}

#[test]
fn thousand_seeded_rotations_round_trip() {
    let mut s: u32 = 0x1234_5678;
    let mut next = || {
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        (s >> 8) as f32 / 8388608.0 - 1.0
    };
    let mut worst = 0.0f32;
    for _ in 0..1000 {
        let q = Quaternion::new(next(), next(), next(), next());
        if q.norm() > 0.1 {
            worst = worst.max(roundtrip(q));
        }
    }
    assert!(worst < 1e-5, "worst {worst}");
}

#[test]
fn matrix_rotates_like_quaternion() {
    let q = Quaternion::new(0.9, 0.1, -0.3, 0.2).normalize().unwrap();
    let m = Dcm::from_quaternion(&q).unwrap().0 .0;
    let v = [0.3, -1.2, 2.0];
    let want = q.rotate_vec(v);
    for i in 0..3 {
        let got = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
        assert!((got - want[i]).abs() < 1e-5);
    }
}

#[test]
fn try_to_quaternion_validates_orthonormality() {
    use stm32_navcore::rotation::RotationError;
    let q = Quaternion::new(0.9, 0.1, -0.3, 0.2).normalize().unwrap();
    let d = Dcm::from_quaternion(&q).unwrap();
    assert!(d.try_to_quaternion().is_ok());
    let mut bad = d;
    bad.0 .0[0][0] *= 1.5;
    assert_eq!(bad.try_to_quaternion(), Err(RotationError::NotOrthonormal));
}

#[test]
fn quaternion_dcm_helper_methods() {
    let q = Quaternion::new(0.9, 0.1, -0.3, 0.2).normalize().unwrap();
    let d = q.to_dcm().unwrap();
    let q_back = Quaternion::from_dcm(&d);
    assert!(max_err(&q, &q_back) < 1e-6);
    let q_try = Quaternion::try_from_dcm(&d).unwrap();
    assert!(max_err(&q, &q_try) < 1e-6);
}

#[test]
fn from_quaternion_normalized_matches_standard() {
    let q = Quaternion::new(0.9, 0.1, -0.3, 0.2).normalize().unwrap();
    let d1 = Dcm::from_quaternion(&q).unwrap();
    let d2 = Dcm::from_quaternion_normalized(&q);
    for i in 0..3 {
        for j in 0..3 {
            assert!((d1.0 .0[i][j] - d2.0 .0[i][j]).abs() < 1e-6);
        }
    }
}

#[test]
fn to_quaternion_handles_all_four_shepperd_branches() {
    let q_w = Quaternion::IDENTITY;
    let q_x = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let q_y = Quaternion::new(0.0, 0.0, 1.0, 0.0);
    let q_z = Quaternion::new(0.0, 0.0, 0.0, 1.0);
    for q in [q_w, q_x, q_y, q_z] {
        let d = Dcm::from_quaternion(&q).unwrap();
        let back = d.to_quaternion();
        assert!(max_err(&q, &back) < 1e-6);
    }
}
