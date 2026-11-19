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
    let cfg = VariometerConfig {
        time_constant_s: -1.0,
        ..Default::default()
    };
    assert_eq!(
        Variometer::new(cfg),
        Err(VariometerError::InvalidTimeConstant)
    );

    cfg.time_constant_s = 1.0;
    cfg.climb_threshold_mps = -2.0;
    cfg.sink_threshold_mps = 1.0;
    assert_eq!(Variometer::new(cfg), Err(VariometerError::InvalidThreshold));
}

#[test]
fn test_altitude_update_and_climb_rate() {
    let mut vario = Variometer::default();
    assert!(!vario.is_initialized());

    let vs = vario.update_altitude(100.0, 0.1).unwrap();
    assert_eq!(vs, 0.0);
    assert!(vario.is_initialized());

    let mut alt = 100.0;
    for _ in 0..100 {
        alt += 2.0 * 0.1;
        vario.update_altitude(alt, 0.1).unwrap();
    }
    assert!((vario.vertical_speed() - 2.0).abs() < 0.05);
    assert!((vario.altitude() - alt).abs() < 0.3);
}

#[test]
fn test_tone_generation_and_pressure_updates() {
    let mut vario = Variometer::default();

    vario.update_pressure(101325.0, 0.1).unwrap();
    assert_eq!(vario.tone(), VarioTone::Silent);

    for _ in 0..50 {
        vario.update_pressure(102000.0, 0.1).unwrap();
    }
    match vario.tone() {
        VarioTone::Sink { freq_hz } => assert!(freq_hz >= 100.0),
        other => panic!("expected Sink tone, got {:?}", other),
    }

    vario.reset();
    assert!(!vario.is_initialized());
}
