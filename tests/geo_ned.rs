use stm32_navcore::geo::{Lla, LtpFrame, Ned, WGS84_A, WGS84_E2};

#[test]
fn test_origin_maps_to_zero_and_roundtrip() {
    let origin = Lla::from_degrees(45.0, 15.0, 200.0).expect("valid coordinates");
    let frame = LtpFrame::from_lla(&origin);

    let ned_origin = frame.ecef_to_ned(&frame.origin_ecef).expect("valid ned");
    assert!(ned_origin.north.abs() < 1e-9);
    assert!(ned_origin.east.abs() < 1e-9);
    assert!(ned_origin.down.abs() < 1e-9);

    let target_ned = Ned::new(123.456, -789.012, 42.0);
    let target_ecef = frame.ned_to_ecef(&target_ned).expect("valid ecef");
    let recovered_ned = frame.ecef_to_ned(&target_ecef).expect("valid ned");
    assert!((recovered_ned.north - target_ned.north).abs() < 1e-9);
    assert!((recovered_ned.east - target_ned.east).abs() < 1e-9);
    assert!((recovered_ned.down - target_ned.down).abs() < 1e-9);
}

#[test]
fn test_points_1km_north_and_east_map_within_bound() {
    let origin = Lla::from_degrees(45.0, 10.0, 100.0).expect("valid coordinates");
    let frame = LtpFrame::from_lla(&origin);

    let sin_lat = libm::sin(origin.lat_rad);
    let m = WGS84_A * (1.0 - WGS84_E2) / libm::pow(1.0 - WGS84_E2 * sin_lat * sin_lat, 1.5);
    let dlat = 1000.0 / m;
    let north_pt = Lla::new(origin.lat_rad + dlat, origin.lon_rad, origin.alt_m).expect("valid");
    let ned_north = frame.lla_to_ned(&north_pt).expect("valid ned");
    assert!((ned_north.north - 1000.0).abs() < 0.5);
    assert!(ned_north.east.abs() < 0.5);

    let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    let dlon = 1000.0 / (n * libm::cos(origin.lat_rad));
    let east_pt = Lla::new(origin.lat_rad, origin.lon_rad + dlon, origin.alt_m).expect("valid");
    let ned_east = frame.lla_to_ned(&east_pt).expect("valid ned");
    assert!(ned_east.north.abs() < 0.5);
    assert!((ned_east.east - 1000.0).abs() < 0.5);
}
