use stm32_navcore::geo::vincenty::inverse;

const D2R: f64 = core::f64::consts::PI / 180.0;

#[test]
fn flinders_peak_to_buninyong() {
    // Vincenty (1975) benchmark: 54972.271 m, azimuth 306 deg 52' 05.37".
    let r = inverse(
        -(37.0 + 57.0 / 60.0 + 3.72030 / 3600.0) * D2R,
        (144.0 + 25.0 / 60.0 + 29.52440 / 3600.0) * D2R,
        -(37.0 + 39.0 / 60.0 + 10.15610 / 3600.0) * D2R,
        (143.0 + 55.0 / 60.0 + 35.38390 / 3600.0) * D2R,
    )
    .unwrap();
    assert!((r.distance_m - 54972.271).abs() < 1e-3);
    let az = (306.0 + 52.0 / 60.0 + 5.37 / 3600.0) * D2R;
    assert!((r.azimuth1 - (az - 2.0 * core::f64::consts::PI)).abs() < 1e-6);
}

#[test]
fn identical_points_give_zero() {
    let r = inverse(0.5, 1.0, 0.5, 1.0).unwrap();
    assert_eq!(r.distance_m, 0.0);
}

#[test]
fn equator_quarter_arc() {
    let r = inverse(0.0, 0.0, 0.0, 90.0 * D2R).unwrap();
    let expected = 6_378_137.0 * core::f64::consts::FRAC_PI_2;
    assert!((r.distance_m - expected).abs() < 1e-3);
}

#[test]
fn antipodal_does_not_panic() {
    let _ = inverse(0.0, 0.0, 0.0, core::f64::consts::PI);
}
