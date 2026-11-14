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
