use stm32_navcore::geo::haversine::{haversine, EARTH_RADIUS_M};
use stm32_navcore::geo::GeoError;

const D2R: f64 = core::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / core::f64::consts::PI;

#[test]
fn identical_points_give_zero_distance() {
    let res = haversine(0.5, 1.0, 0.5, 1.0).unwrap();
    assert_eq!(res.distance_m, 0.0);
}
