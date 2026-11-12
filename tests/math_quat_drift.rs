use stm32_navcore::math::{MathError, Quaternion};

#[test]
fn norm_stays_unit_after_long_integration() {
    let mut q = Quaternion::IDENTITY;
    for i in 0..100_000 {
        let k = (i % 7) as f32 * 0.1;
        q = q.integrate_gyro([0.3 + k, -0.2, 0.5 - k], 0.005).unwrap();
    }
    assert!((q.norm() - 1.0).abs() < 1e-5);
}

#[test]
fn normalize_corrects_unnormalized_product_drift() {
    let step = Quaternion::new(0.9999, 0.01, 0.0, 0.0);
    let mut q = Quaternion::IDENTITY;
    for _ in 0..1000 {
        q = q.mul(&step);
    }
    assert!((q.norm() - 1.0).abs() > 1e-4);
    let n = q.normalize().unwrap();
    assert!((n.norm() - 1.0).abs() < 1e-6);
}

#[test]
fn invalid_norms_are_rejected() {
    let zero = Quaternion::new(0.0, 0.0, 0.0, 0.0);
    assert_eq!(zero.normalize(), Err(MathError::InvalidNorm));
    let nan = Quaternion::new(f32::NAN, 0.0, 0.0, 0.0);
    assert_eq!(nan.normalize(), Err(MathError::InvalidNorm));
    let q = Quaternion::IDENTITY;
    assert_eq!(
        q.integrate_gyro([f32::NAN, 0.0, 0.0], 0.01),
        Err(MathError::InvalidNorm)
    );
    assert_eq!(
        q.integrate_gyro([0.0; 3], -0.01),
        Err(MathError::InvalidNorm)
    );
}
