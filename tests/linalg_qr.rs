use core::f32;

fn check_givens_rotations(matrix: &[[f32; 4]; 6]) {
    let mut temp_matrix = matrix.clone();
    givens_rotations(&mut temp_matrix);
    for i in 0..5 {
        let x = temp_matrix[i][i];
        let y = temp_matrix[i + 1][i];

        if y.abs() > x.abs() {
            assert!(y / (x * x + y * y).sqrt().abs() < 1e-5);
        } else {
            assert!(x / (x * x + y * y).sqrt().abs() < 1e-5);
        }
    }
}

fn check_qr_decomposition(matrix: &[[f32; 4]; 6]) {
    let mut temp_matrix = matrix.clone();
    qr_decomposition(&mut temp_matrix);
    for i in 0..5 {
        let x = temp_matrix[i][i];
        let y = temp_matrix[i + 1][i];

