use stm32_navcore::geo::{initial_bearing_deg, initial_bearing_rad, GeoError};

#[test]
fn test_cardinal_bearings() {
    let east = initial_bearing_deg(0.0, 0.0, 0.0, 10.0).unwrap();
    assert!((east - 90.0).abs() < 1e-6);

    let north = initial_bearing_deg(0.0, 0.0, 10.0, 0.0).unwrap();
    assert!((north - 0.0).abs() < 1e-6);

    let west = initial_bearing_deg(0.0, 10.0, 0.0, 0.0).unwrap();
    assert!((west - 270.0).abs() < 1e-6);

    let south = initial_bearing_deg(10.0, 0.0, 0.0, 0.0).unwrap();
    assert!((south - 180.0).abs() < 1e-6);
}
