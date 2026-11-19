use stm32_navcore::filters::slew::{SlewError, SlewRateLimiter, SymmetricDeadband};

#[test]
fn test_slew_rate_limiter_and_deadband() {
    let mut limiter = SlewRateLimiter::new(10.0, 0.1).expect("valid parameters");
    assert_eq!(limiter.update(0.0), 0.0);
    assert_eq!(limiter.update(5.0), 1.0);
    assert_eq!(limiter.update(5.0), 2.0);

    let deadband = SymmetricDeadband::new(0.5).expect("valid threshold");
    assert_eq!(deadband.apply(0.2), 0.0);
    assert_eq!(deadband.apply(-0.2), 0.0);
    assert_eq!(deadband.apply(1.0), 0.5);
    assert_eq!(deadband.apply(-1.0), -0.5);

    assert_eq!(
        SlewRateLimiter::new(-1.0, 0.1).err(),
        Some(SlewError::InvalidParameter)
    );
    assert_eq!(
        SymmetricDeadband::new(-0.5).err(),
        Some(SlewError::InvalidParameter)
    );
}
