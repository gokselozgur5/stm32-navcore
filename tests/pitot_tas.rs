use stm32_navcore::pitot_tas::*;

#[test]
fn test_standard_sea_level_density_and_ratio() {
    let rho = air_density(
        STANDARD_SEA_LEVEL_PRESSURE_PA,
        STANDARD_SEA_LEVEL_TEMP_K,
    )
    .unwrap();
    assert!((rho - STANDARD_SEA_LEVEL_DENSITY_KG_M3).abs() < 1e-3);

    let sigma = density_ratio(
        STANDARD_SEA_LEVEL_PRESSURE_PA,
        STANDARD_SEA_LEVEL_TEMP_K,
    )
    .unwrap();
    assert!((sigma - 1.0).abs() < 1e-4);

    let k = celsius_to_kelvin(15.0).unwrap();
    assert!((k - STANDARD_SEA_LEVEL_TEMP_K).abs() < 1e-3);
}
