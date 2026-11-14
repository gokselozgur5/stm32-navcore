use stm32_navcore::math::{Mat3, Vec3};
use stm32_navcore::rotation::orthonormalize::{dcm, restore_orthonormality};

#[test]
fn test_restore_orthonormality() {
    let vec = Vec3::new(1.0, 2.0, 3.0);
    let dcm = dcm(&vec);
    let restored_dcm = restore_orthonormality(&dcm);

    // Check if the restored DCM is close to the original DCM
    for i in 0..3 {
        for j in 0..3 {
            assert!((restored_dcm[i][j] - dcm[i][j]).abs() < 1e-6);
        }
    }

    // Check if the restored DCM is orthogonal
    let det = restored_dcm.determinant();
    assert!(det.abs() > 0.999); // The determinant should be close to 1
}
