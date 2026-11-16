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

#[test]
fn test_declination_polynomial_eval() {
    use stm32_navcore::wmm_declination::{DeclinationError, DeclinationPolynomial};
    let poly = DeclinationPolynomial::new(
        0.0, 0.0, 90.0, 180.0,
        [5.0, 2.0, -1.0, 0.5, 0.0, -0.2],
    );
    let val = poly.eval(0.0, 0.0).unwrap();
    assert_eq!(val, 5.0);

    let val_pt = poly.eval(45.0, 90.0).unwrap();
    let expected = 5.0 + 2.0 * 0.5 - 1.0 * 0.5 + 0.5 * 0.25 - 0.2 * 0.25;
    assert!((val_pt - expected).abs() < 1e-6);

    let zero_scale = DeclinationPolynomial::new(0.0, 0.0, 0.0, 180.0, [0.0; 6]);
    assert_eq!(zero_scale.eval(0.0, 0.0), Err(DeclinationError::InvalidScale));
}

#[test]
fn test_wmm_global_declination_and_bounds() {
    use stm32_navcore::wmm_declination::{
        wmm_declination_deg, wmm_declination_rad, DeclinationError,
    };
    let eq = wmm_declination_deg(0.0, 0.0).unwrap();
    assert_eq!(eq, -3.5);

    let rad = wmm_declination_rad(0.0, 0.0).unwrap();
    let expected_rad = -3.5 * core::f32::consts::PI / 180.0;
    assert!((rad - expected_rad).abs() < 1e-6);

    assert_eq!(wmm_declination_deg(95.0, 0.0), Err(DeclinationError::InvalidLatitude));
    assert_eq!(wmm_declination_deg(0.0, 200.0), Err(DeclinationError::InvalidLongitude));
    assert_eq!(wmm_declination_deg(f32::NAN, 0.0), Err(DeclinationError::NonFinite));
}
