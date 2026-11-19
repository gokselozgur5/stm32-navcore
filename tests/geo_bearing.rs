use stm32_navcore::geo::{initial_bearing_deg, initial_bearing_rad, GeoError};

#[test]
fn test_cardinal_bearings() {
    let east = initial_bearing_deg(0.0, 0.0, 0.0, 10.0).unwrap();
    // f32 precision tolerance (~1e-5 rad)
    assert!((east - 90.0).abs() < 1e-4);

    let north = initial_bearing_deg(0.0, 0.0, 10.0, 0.0).unwrap();
    // f32 precision tolerance (~1e-5 rad)
    assert!((north - 0.0).abs() < 1e-4);

    let west = initial_bearing_deg(0.0, 10.0, 0.0, 0.0).unwrap();
    // f32 precision tolerance (~1e-5 rad)
    assert!((west - 270.0).abs() < 1e-4);

    let south = initial_bearing_deg(10.0, 0.0, 0.0, 0.0).unwrap();
    // f32 precision tolerance (~1e-5 rad)
    assert!((south - 180.0).abs() < 1e-4);
}

#[test]
fn test_bearing_validation_errors() {
    assert_eq!(
        initial_bearing_deg(f32::NAN, 0.0, 0.0, 0.0),
        Err(GeoError::NonFinite)
    );
    assert_eq!(
        initial_bearing_deg(95.0, 0.0, 0.0, 0.0),
        Err(GeoError::InvalidLatitude)
    );
    assert_eq!(
        initial_bearing_deg(-95.0, 0.0, 0.0, 0.0),
        Err(GeoError::InvalidLatitude)
    );
    assert_eq!(
        initial_bearing_deg(0.0, 0.0, 95.0, 0.0),
        Err(GeoError::InvalidLatitude)
    );
    assert_eq!(
        initial_bearing_rad(f32::INFINITY, 0.0, 0.0, 0.0),
        Err(GeoError::NonFinite)
    );
}

#[test]
fn test_bearing_f32_suite_compiles() {
    let deg = initial_bearing_deg(0.0_f32, 0.0_f32, 10.0_f32, 0.0_f32).unwrap();
    // f32 tolerance ~1e-5 rad (~5.7e-4 deg)
    assert!((deg - 0.0_f32).abs() < 1e-3);
}
