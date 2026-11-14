use stm32_navcore::geo::{lla_to_ecef, Lla, WGS84_A, WGS84_B};

#[test]
fn test_equator_prime_meridian() {
    let lla = Lla::new(0.0, 0.0, 0.0).expect("valid coordinates");
    let ecef = lla_to_ecef(&lla);
    assert!((ecef.x - WGS84_A).abs() < 1e-4);
    assert!(ecef.y.abs() < 1e-4);
    assert!(ecef.z.abs() < 1e-4);
}

#[test]
fn test_north_pole() {
    let half_pi = core::f64::consts::FRAC_PI_2;
    let lla = Lla::new(half_pi, 0.0, 0.0).expect("valid coordinates");
    let ecef = lla_to_ecef(&lla);
    assert!(ecef.x.abs() < 1e-4);
    assert!(ecef.y.abs() < 1e-4);
    assert!((ecef.z - WGS84_B).abs() < 1e-4);
}
