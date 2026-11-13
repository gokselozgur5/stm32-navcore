use stm32_navcore::math::Quaternion;
use stm32_navcore::rotation::so3_exp::{so3_exp, so3_log};

const AXES: [[f32; 3]; 4] = [
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.6, 0.0, 0.8],
    [0.48, -0.64, 0.6],
];

fn worst_error(angle: f32) -> f32 {
    let mut worst = 0.0f32;
    for a in AXES {
        let v = [a[0] * angle, a[1] * angle, a[2] * angle];
        let back = so3_log(&so3_exp(v)).unwrap();
        for i in 0..3 {
            worst = worst.max((back[i] - v[i]).abs());
        }
    }
    worst
}

#[test]
fn zero_vector_maps_to_identity() {
    assert_eq!(so3_exp([0.0; 3]), Quaternion::IDENTITY);
    assert_eq!(so3_log(&Quaternion::IDENTITY).unwrap(), [0.0; 3]);
}
