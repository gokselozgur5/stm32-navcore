//! Integration tests for WGS-84 normal gravity and spherical harmonics.

use stm32_navcore::gravity_wgs84::*;

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
