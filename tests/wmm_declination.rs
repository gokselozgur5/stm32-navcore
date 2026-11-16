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

#[test]
fn test_magnetic_true_heading_roundtrip() {
    use stm32_navcore::wmm_declination::{
        magnetic_to_true_heading_deg, true_to_magnetic_heading_deg,
    };
    let mag = 350.0;
    let dec_east = 15.0;
    let true_hdg = magnetic_to_true_heading_deg(mag, dec_east).unwrap();
    assert_eq!(true_hdg, 5.0);
    let recovered_mag = true_to_magnetic_heading_deg(true_hdg, dec_east).unwrap();
    assert_eq!(recovered_mag, 350.0);

    let dec_west = -10.0;
    let true_hdg_w = magnetic_to_true_heading_deg(5.0, dec_west).unwrap();
    assert_eq!(true_hdg_w, 355.0);
}
