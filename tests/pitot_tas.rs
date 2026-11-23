use stm32_navcore::pitot_tas::*;

#[test]
fn test_standard_sea_level_density_and_ratio() {
    let rho = air_density(STANDARD_SEA_LEVEL_PRESSURE_PA, STANDARD_SEA_LEVEL_TEMP_K).unwrap();
    assert!((rho - STANDARD_SEA_LEVEL_DENSITY_KG_M3).abs() < 1e-3);

    let sigma = density_ratio(STANDARD_SEA_LEVEL_PRESSURE_PA, STANDARD_SEA_LEVEL_TEMP_K).unwrap();
    assert!((sigma - 1.0).abs() < 1e-4);

    let k = celsius_to_kelvin(15.0).unwrap();
    assert!((k - STANDARD_SEA_LEVEL_TEMP_K).abs() < 1e-3);
}

#[test]
fn test_pitot_invalid_inputs_rejected() {
    assert_eq!(air_density(f32::NAN, 288.15), Err(PitotError::NonFinite));
    assert_eq!(
        air_density(101325.0, -5.0),
        Err(PitotError::InvalidTemperature)
    );
    assert_eq!(
        air_density(-100.0, 288.15),
        Err(PitotError::InvalidPressure)
    );

    assert_eq!(
        diff_pressure_to_ias(-10.0),
        Err(PitotError::NegativeDifferentialPressure)
    );
    assert_eq!(diff_pressure_to_ias(f32::NAN), Err(PitotError::NonFinite));

    assert_eq!(
        ias_to_tas(-1.0, 101325.0, 288.15),
        Err(PitotError::NegativeAirspeed)
    );
    assert_eq!(
        ias_to_tas_with_density(10.0, -1.0),
        Err(PitotError::InvalidDensity)
    );
}

#[test]
fn test_ias_to_tas_altitude_scaling() {
    let p_sea = 101325.0;
    let t_sea = 288.15;
    let tas_sea = ias_to_tas(50.0, p_sea, t_sea).unwrap();
    assert!((tas_sea - 50.0).abs() < 0.05);

    let p_5000m = 54020.0;
    let t_5000m = 255.65;
    let tas_5000m = ias_to_tas(50.0, p_5000m, t_5000m).unwrap();
    assert!(tas_5000m > 50.0);
    assert!((tas_5000m - 64.6).abs() < 1.0);
}

#[test]
fn test_diff_pressure_conversions_and_estimator() {
    let dp = 122.5;
    let ias = diff_pressure_to_ias(dp).unwrap();
    assert!((ias - 14.142).abs() < 0.05);

    let est = PitotTasEstimator::default();
    let r = est
        .update(
            dp,
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            STANDARD_SEA_LEVEL_TEMP_K,
        )
        .unwrap();
    assert!((r.ias_mps - 14.142).abs() < 0.05);
    assert!((r.tas_mps - 14.142).abs() < 0.05);
    assert!((r.dynamic_pressure_pa - dp).abs() < 1e-4);

    assert_eq!(
        PitotTasEstimator::with_factor(-1.0),
        Err(PitotError::InvalidFactor)
    );
}
