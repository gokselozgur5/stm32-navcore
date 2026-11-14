use core::f64::consts::FRAC_PI_4;
use stm32_navcore::geo::earth_rate::earth_rotation_rate;
use stm32_navcore::geo::WGS84_OMEGA_EARTH;

#[test]
fn earth_rate_at_45_deg_matches_omega_cos_sin() {
    let rate = earth_rotation_rate(FRAC_PI_4).unwrap();
    let expected_n = WGS84_OMEGA_EARTH * libm::cos(FRAC_PI_4);
    let expected_d = -WGS84_OMEGA_EARTH * libm::sin(FRAC_PI_4);
    assert!((rate.north - expected_n).abs() < 1e-9);
    assert_eq!(rate.east, 0.0);
    assert!((rate.down - expected_d).abs() < 1e-9);
}

use stm32_navcore::geo::earth_rate::{principal_radii_of_curvature, transport_rate};

#[test]
fn transport_rate_matches_v_over_r() {
    let lat = 0.0;
    let (r_m, r_n) = principal_radii_of_curvature(lat).unwrap();
    let rate = transport_rate(lat, 0.0, 100.0, 50.0).unwrap();
    assert!((rate.north - 50.0 / r_n).abs() < 1e-12);
    assert!((rate.east - (-100.0 / r_m)).abs() < 1e-12);
    assert!(rate.down.abs() < 1e-12);
}
