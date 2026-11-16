use stm32_navcore::wmm_declination::DeclinationError;

#[test]
fn test_declination_error_variants() {
    assert_eq!(DeclinationError::NonFinite, DeclinationError::NonFinite);
    assert_ne!(DeclinationError::NonFinite, DeclinationError::InvalidLatitude);
    assert_ne!(DeclinationError::InvalidLatitude, DeclinationError::InvalidLongitude);
    assert_ne!(DeclinationError::InvalidLongitude, DeclinationError::InvalidScale);
}

#[test]
fn test_heading_normalization() {
    use stm32_navcore::wmm_declination::{normalize_heading_deg, normalize_heading_rad};
    assert_eq!(normalize_heading_deg(0.0), 0.0);
    assert_eq!(normalize_heading_deg(360.0), 0.0);
    assert_eq!(normalize_heading_deg(-90.0), 270.0);
    assert_eq!(normalize_heading_deg(450.0), 90.0);
    let pi = core::f32::consts::PI;
    assert_eq!(normalize_heading_rad(0.0), 0.0);
    assert!((normalize_heading_rad(-pi / 2.0) - 1.5 * pi).abs() < 1e-6);
}
