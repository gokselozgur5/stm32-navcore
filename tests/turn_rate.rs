use stm32_navcore::turn_rate::*;

#[test]
fn test_turn_rate_error_variants() {
    assert_eq!(
        turn_rate_rad_s(f32::NAN, 0.2, STANDARD_GRAVITY_MPS2),
        Err(TurnRateError::NonFinite)
    );
    assert_eq!(
        turn_rate_rad_s(-10.0, 0.2, STANDARD_GRAVITY_MPS2),
        Err(TurnRateError::InvalidAirspeed)
    );
    assert_eq!(
        turn_rate_rad_s(50.0, 0.2, -9.8),
        Err(TurnRateError::InvalidGravity)
    );
    assert_eq!(
        turn_rate_rad_s(50.0, 1.55, STANDARD_GRAVITY_MPS2),
        Err(TurnRateError::BankAngleOutOfRange)
    );
}
