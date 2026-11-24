//! Integration tests for WGS-84 normal gravity and spherical harmonics.

use stm32_navcore::geo::gravity_wgs84::*;

#[test]
fn test_wgs84_equator_polar_gravity() {
    let eq = somigliana_surface(0.0).unwrap();
    assert!((eq - GAMMA_E).abs() < 1e-6);

    let half_pi = core::f64::consts::FRAC_PI_2;
    let np = somigliana_surface(half_pi).unwrap();
    assert!((np - GAMMA_P).abs() < 1e-6);

    let sp = somigliana_surface(-half_pi).unwrap();
    assert!((sp - GAMMA_P).abs() < 1e-6);
}

#[test]
fn test_wgs84_elevation_gradient() {
    let lat_45 = core::f64::consts::FRAC_PI_4;
    let g0 = normal_gravity_elevation(lat_45, 0.0).unwrap();
    let g5000 = normal_gravity_elevation(lat_45, 5000.0).unwrap();
    assert!(g5000 < g0);
    let delta = g0 - g5000;
    assert!((delta - 5.0 * 0.003086).abs() < 0.01);

    let g_f32 = normal_gravity_elevation_f32(lat_45 as f32, 5000.0f32).unwrap();
    assert!((g_f32 - (g5000 as f32)).abs() < 1e-4);
}

#[test]
fn test_wgs84_harmonics_and_ecef_gravity() {
    let sh = spherical_harmonic_gravity(WGS84_A, 0.0).unwrap();
    assert!(sh.radial > 9.78 && sh.radial < 9.85);

    let ecef = effective_gravity_ecef([WGS84_A, 0.0, 0.0]).unwrap();
    let norm = libm::sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1] + ecef[2] * ecef[2]);
    assert!((norm - GAMMA_E).abs() < 0.05);

    let calc = Wgs84GravityCalculator::new();
    let ned = calc.ned(0.0, 0.0).unwrap();
    assert_eq!(ned[0], 0.0);
    assert_eq!(ned[1], 0.0);
    assert!((ned[2] - GAMMA_E).abs() < 1e-6);
}
