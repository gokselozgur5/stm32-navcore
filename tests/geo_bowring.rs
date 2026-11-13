use stm32_navcore::geo::{ecef_to_lla, lla_to_ecef, Lla};

#[test]
fn test_equator_and_pole_roundtrip() {
    let eq = Lla::new(0.0, 0.0, 0.0).expect("valid coordinates");
    let eq_ecef = lla_to_ecef(&eq);
    let eq_inv = ecef_to_lla(&eq_ecef).expect("valid inverse");
    assert!((eq_inv.lat_rad - eq.lat_rad).abs() < 1e-9);
    assert!((eq_inv.lon_rad - eq.lon_rad).abs() < 1e-9);
    assert!((eq_inv.alt_m - eq.alt_m).abs() < 1e-3);

    let half_pi = core::f64::consts::FRAC_PI_2;
    let pole = Lla::new(half_pi, 0.0, 100.0).expect("valid coordinates");
    let pole_ecef = lla_to_ecef(&pole);
    let pole_inv = ecef_to_lla(&pole_ecef).expect("valid inverse");
    assert!((pole_inv.lat_rad - pole.lat_rad).abs() < 1e-9);
    assert!((pole_inv.alt_m - pole.alt_m).abs() < 1e-3);
}
