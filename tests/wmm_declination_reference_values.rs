use stm32_navcore::wmm_declination::wmm_declination_deg;

// WMM2020 report test values, epoch 2020.0, altitude 0 km:
//   lat 0, lon 120 E    -> D = 0.16 deg
//   lat 80 S, lon 240 E -> D = 69.36 deg
#[test]
fn global_declination_matches_wmm2020_test_values() {
    let d1 = wmm_declination_deg(0.0, 120.0).unwrap();
    let d2 = wmm_declination_deg(-80.0, -120.0).unwrap();
    let east = wmm_declination_deg(10.0, 180.0).unwrap();
    let west = wmm_declination_deg(10.0, -180.0).unwrap();
    assert!((d1 - 0.16).abs() < 5.0, "declination at 0N 120E = {d1}");
    assert!((d2 - 69.36).abs() < 5.0, "declination at 80S 240E = {d2}");
    assert!(
        (east - west).abs() < 1.0e-3,
        "antimeridian jump = {}",
        east - west
    );
}
