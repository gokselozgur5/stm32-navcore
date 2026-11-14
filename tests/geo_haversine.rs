use stm32_navcore::geo::haversine::{haversine, EARTH_RADIUS_M};
use stm32_navcore::geo::GeoError;

const D2R: f64 = core::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / core::f64::consts::PI;

#[test]
fn identical_points_give_zero_distance() {
    let res = haversine(0.5, 1.0, 0.5, 1.0).unwrap();
    assert_eq!(res.distance_m, 0.0);
}

#[test]
fn equator_quarter_arc() {
    let res = haversine(0.0, 0.0, 0.0, 90.0 * D2R).unwrap();
    let expected = EARTH_RADIUS_M * core::f64::consts::FRAC_PI_2;
    assert!((res.distance_m - expected).abs() < 1e-6);
    assert!((res.bearing_rad - core::f64::consts::FRAC_PI_2).abs() < 1e-6);
}

#[test]
fn london_to_paris_accuracy() {
    let london_lat = 51.5074 * D2R;
    let london_lon = -0.1278 * D2R;
    let paris_lat = 48.8566 * D2R;
    let paris_lon = 2.3522 * D2R;
    let res = haversine(london_lat, london_lon, paris_lat, paris_lon).unwrap();
    let published_dist = 343_923.0;
    let dist_err_pct = (res.distance_m - published_dist).abs() / published_dist * 100.0;
    assert!(dist_err_pct < 0.5);
    let published_bearing_deg = 148.046;
    let bearing_deg = res.bearing_rad * R2D;
    assert!((bearing_deg - published_bearing_deg).abs() < 0.1);
}

#[test]
fn new_york_to_london_accuracy() {
    let ny_lat = 40.7128 * D2R;
    let ny_lon = -74.0060 * D2R;
    let london_lat = 51.5074 * D2R;
    let london_lon = -0.1278 * D2R;
    let res = haversine(ny_lat, ny_lon, london_lat, london_lon).unwrap();
    let published_dist = 5_585_234.0;
    let dist_err_pct = (res.distance_m - published_dist).abs() / published_dist * 100.0;
    assert!(dist_err_pct < 0.5);
    let published_bearing_deg = 51.241;
    let bearing_deg = res.bearing_rad * R2D;
    assert!((bearing_deg - published_bearing_deg).abs() < 0.1);
}

#[test]
fn invalid_inputs_are_rejected() {
    assert_eq!(haversine(f64::NAN, 0.0, 0.0, 0.0), Err(GeoError::NonFinite));
    assert_eq!(
        haversine(2.0, 0.0, 0.0, 0.0),
        Err(GeoError::InvalidLatitude)
    );
}
