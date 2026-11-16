use stm32_navcore::wmm_declination::DeclinationError;

#[test]
fn test_declination_error_variants() {
    assert_eq!(DeclinationError::NonFinite, DeclinationError::NonFinite);
    assert_ne!(DeclinationError::NonFinite, DeclinationError::InvalidLatitude);
    assert_ne!(DeclinationError::InvalidLatitude, DeclinationError::InvalidLongitude);
    assert_ne!(DeclinationError::InvalidLongitude, DeclinationError::InvalidScale);
}
