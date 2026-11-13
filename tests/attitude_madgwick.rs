use stm32_navcore::attitude::Madgwick;
use stm32_navcore::math::Quaternion;

#[test]
fn test_madgwick_initialization() {
    let mut f = Madgwick::default();
    assert_eq!(f.quaternion(), Quaternion::IDENTITY);
    assert!((f.beta - Madgwick::DEFAULT_BETA).abs() < 1e-6);

    let custom = Madgwick::new(0.05);
    assert!((custom.beta - 0.05).abs() < 1e-6);

    let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    f.set_quaternion(q).unwrap();
    assert_eq!(f.quaternion(), q);
    f.reset();
    assert_eq!(f.quaternion(), Quaternion::IDENTITY);
}
