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
/// * `vec` - A 3D vector representing the rotation axis.
///
/// # Returns
///
/// * A 3x3 DCM matrix.
fn dcm(vec: &Vec<f32>) -> Mat3 {
    let skew = skew_matrix(vec);
    let mut dcm = Mat3::identity();
    for i in 0..3 {
        for j in 0..3 {
            if i != j {
                let cross = skew.cross(&vec[i]);
                dcm[(i, j)] += cross;
            }
        }
    }
    dcm
}

/// Restores a DCM perturbed by 1e-3 noise to orthogonality error below 1e-6.
///
/// # Arguments
///
/// * `dcm` - A 3x3 DCM matrix.
