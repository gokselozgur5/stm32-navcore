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

use stm32_navcore::{NedFrame, NedPos};

#[test]
fn test_integration_ned_frame_displacement() {
    let base = GeodeticPos::from_degrees(48.8566, 2.3522, 35.0).expect("valid Paris coords");
    let frame = NedFrame::from_geodetic(&base);
    let delta = NedPos::new(200.0, 300.0, -50.0);
    let ecef = frame.ned_to_ecef(&delta).expect("valid ecef");
    let back_ned = frame.ecef_to_ned(&ecef).expect("valid ned");
    assert!((back_ned.north - delta.north).abs() < 1e-4);
    assert!((back_ned.east - delta.east).abs() < 1e-4);
    assert!((back_ned.down - delta.down).abs() < 1e-4);
}
