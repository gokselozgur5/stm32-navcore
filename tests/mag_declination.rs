use stm32_navcore::mag_declination::{estimate_wmm2020, MagDeclinationError};

#[test]
fn test_wmm_invalid_inputs() {
    assert_eq!(
        estimate_wmm2020(f32::NAN, 0.0),
        Err(MagDeclinationError::NonFinite)
    );
    assert_eq!(
        estimate_wmm2020(0.0, f32::NAN),
        Err(MagDeclinationError::NonFinite)
    );
    assert_eq!(
        estimate_wmm2020(91.0, 0.0),
        Err(MagDeclinationError::InvalidLatitude)
    );
    assert_eq!(
        estimate_wmm2020(-91.0, 0.0),
        Err(MagDeclinationError::InvalidLatitude)
    );
    assert_eq!(
        estimate_wmm2020(0.0, 361.0),
        Err(MagDeclinationError::InvalidLongitude)
    );
}
