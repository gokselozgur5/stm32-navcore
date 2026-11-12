use stm32_navcore::math::Quaternion;

fn axis_angle(ax: [f32; 3], ang: f32) -> Quaternion {
    let n = (ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]).sqrt();
    let s = (ang * 0.5).sin() / n;
    Quaternion::new((ang * 0.5).cos(), ax[0] * s, ax[1] * s, ax[2] * s)
}

fn len(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

#[test]
fn rotation_preserves_vector_length() {
    let v = [1.5, -2.0, 0.7];
    for i in 1..40 {
        let q = axis_angle([0.3, 1.0, -0.6], i as f32 * 0.15);
        assert!((len(q.rotate_vec(v)) - len(v)).abs() < 1e-5);
    }
}
