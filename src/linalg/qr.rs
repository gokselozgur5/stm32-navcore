use core::f32;

/// Perform Givens rotations on a matrix to make it upper triangular.
///
/// # Arguments
///
/// * `matrix` - A mutable 2D array representing the matrix to be decomposed.
fn givens_rotations(matrix: &mut [[f32; 4]; 6]) {
    for i in 0..5 {
        let mut x = matrix[i][i];
        let y = matrix[i + 1][i];

        if y.abs() > x.abs() {
            let c = y / (x * x + y * y).sqrt();
            let s = -c * x;
            for j in i..6 {
                let temp = matrix[j][i];
                matrix[j][i] = c * temp + s * matrix[j + 1][i];
                matrix[j + 1][i] = -s * temp + c * matrix[j + 1][i];
            }
        } else {
            let c = x / (x * x + y * y).sqrt();
            let s = -c * y;
            for j in i..6 {
                let temp = matrix[j][i];
                matrix[j][i] = c * temp + s * matrix[j + 1][i];
                matrix[j + 1][i] = -s * temp + c * matrix[j + 1][i];
            }
        }
    }
}

/// Perform QR decomposition on a matrix to make it upper triangular.
///
/// # Arguments
///
/// * `matrix` - A mutable 2D array representing the matrix to be decomposed.
fn qr_decomposition(matrix: &mut [[f32; 4]; 6]) {
    for i in 0..5 {
        let mut x = matrix[i][i];
        let y = matrix[i + 1][i];

        if y.abs() > x.abs() {
            let c = y / (x * x + y * y).sqrt();
            let s = -c * x;
            for j in i..6 {
                let temp = matrix[j][i];
                matrix[j][i] = c * temp + s * matrix[j + 1][i];
                matrix[j + 1][i] = -s * temp + c * matrix[j + 1][i];
            }
        } else {
            let c = x / (x * x + y * y).sqrt();
            let s = -c * y;
            for j in i..6 {
                let temp = matrix[j][i];
                matrix[j][i] = c * temp + s * matrix[j + 1][i];
                matrix[j + 1][i] = -s * temp + c * matrix[j + 1][i];
            }
        }
    }
}
