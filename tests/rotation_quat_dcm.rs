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
