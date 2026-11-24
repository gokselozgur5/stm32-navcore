use stm32_navcore::airdata::baro_alt::BaroAltEstimator;

#[test]
fn calibration_reproduces_known_altitude_with_custom_reference() {
    let mut est = BaroAltEstimator::with_custom_reference(101_325.0, 303.15, 0.0065).unwrap();
    est.calibrate_at_known_altitude(95_000.0, 500.0).unwrap();
    let alt = est.estimate_altitude(95_000.0).unwrap();
    assert!(
        (alt - 500.0).abs() < 1.0,
        "altitude after calibration = {alt}"
    );
}
