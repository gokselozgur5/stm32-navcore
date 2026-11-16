use stm32_navcore::variometer::*;

#[test]
fn test_unit_conversions_and_roundtrip() {
    let mps = 5.0;
    let fpm = mps_to_fpm(mps);
    assert!((fpm - 984.252).abs() < 1e-2);
    assert!((fpm_to_mps(fpm) - mps).abs() < 1e-4);

    let knots = mps_to_knots(mps);
    assert!((knots - 9.71922).abs() < 1e-2);
    assert!((knots_to_mps(knots) - mps).abs() < 1e-4);
}

#[test]
fn test_total_energy_altitude() {
    let static_alt = 1000.0;
    let tas = 20.0;
    let te_alt = total_energy_altitude(static_alt, tas).unwrap();
    assert!((te_alt - 1020.3942).abs() < 1e-2);

    assert_eq!(
        total_energy_altitude(f32::NAN, tas),
        Err(VariometerError::NonFinite)
    );
}

#[test]
fn test_config_validation_and_errors() {
    let mut cfg = VariometerConfig::default();
    cfg.time_constant_s = -1.0;
    assert_eq!(
        Variometer::new(cfg),
        Err(VariometerError::InvalidTimeConstant)
    );

    cfg.time_constant_s = 1.0;
    cfg.climb_threshold_mps = -2.0;
    cfg.sink_threshold_mps = 1.0;
    assert_eq!(
        Variometer::new(cfg),
        Err(VariometerError::InvalidThreshold)
    );
}
