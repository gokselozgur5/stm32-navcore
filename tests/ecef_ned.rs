use stm32_navcore::{
    ecef_to_geodetic, geodetic_to_ecef, GeodeticPos,
};

#[test]
fn test_integration_public_api_types_and_reexports() {
    let geo = GeodeticPos::from_degrees(0.0, 0.0, 0.0).expect("valid coordinates");
    let ecef = geodetic_to_ecef(&geo);
    let recovered_geo = ecef_to_geodetic(&ecef).expect("valid ecef");
    assert!((recovered_geo.lat_deg() - 0.0).abs() < 1e-7);
    assert!((recovered_geo.lon_deg() - 0.0).abs() < 1e-7);
    assert!(recovered_geo.alt_m.abs() < 1e-3);
}
