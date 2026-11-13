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

#[test]
fn test_roundtrip_heights_and_latitudes() {
    for lat_deg in [-80.0, -45.0, 0.0, 35.0, 60.0, 85.0] {
        for lon_deg in [-170.0, -90.0, 0.0, 45.0, 120.0] {
            for alt_m in [-500.0, 0.0, 500.0, 5000.0, 20000.0] {
                let lla = Lla::from_degrees(lat_deg, lon_deg, alt_m).expect("valid coordinates");
                let ecef = lla_to_ecef(&lla);
                let recovered = ecef_to_lla(&ecef).expect("valid inverse");
                assert!((recovered.lat_rad - lla.lat_rad).abs() < 1e-6);
                assert!((recovered.lon_rad - lla.lon_rad).abs() < 1e-6);
                assert!((recovered.alt_m - lla.alt_m).abs() < 1.0);
            }
        }
    }
}
