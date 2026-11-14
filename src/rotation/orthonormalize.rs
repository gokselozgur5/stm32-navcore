use std::f32;

/// Computes the skew matrix of a rotation vector.
///
/// # Arguments
///
/// * `vec` - A 3D vector representing the rotation axis.
///
/// # Returns
///
/// * A 3x3 skew matrix.
fn skew_matrix(vec: &Vec<f32>) -> Mat3 {
    let [x, y, z] = vec;
    Mat3::new(
        0.0, -z, y,
         z, 0.0, -x,
        -y, x, 0.0
    )
}

/// Computes the DCM (Direction Cosine Matrix) of a rotation vector.
///
/// # Arguments
///
