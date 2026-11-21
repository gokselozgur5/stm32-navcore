use stm32_navcore::{airspeed, baro_alt, hypsometric, math, pitot_tas};

#[test]
fn test_cross_module_isa_pressure_altitude_consistency() {
    let p_pa = 70000.0;
    let p_hpa = 700.0;
    let alt_math = math::pressure_hpa_to_altitude_m(p_hpa, 1013.25).unwrap();
    let alt_baro = baro_alt::altitude_from_pressure_isa(p_pa).unwrap();
    let alt_airspeed = airspeed::pressure_altitude(p_pa).unwrap();

    assert!((alt_math - alt_baro).abs() < 1e-4);
    assert!((alt_airspeed - alt_baro).abs() < 1e-4);
}

#[test]
fn test_cross_module_tas_consistency() {
    let (diff_p, p0, temp) = (6000.0, 101325.0, 288.15);
    let tas_airspeed = airspeed::true_airspeed(diff_p, p0, temp).unwrap();
    let tas_pitot = pitot_tas::diff_pressure_to_tas(diff_p, p0, temp).unwrap();

    assert!((tas_pitot - tas_airspeed).abs() < 1e-4);
}

#[test]
fn test_cross_module_lapse_rate_consistency() {
    let excessive_lapse = 0.06;
    let res_baro = baro_alt::hypsometric_altitude(70000.0, 101325.0, 288.15, excessive_lapse);
    let res_hyp = hypsometric::hypsometric_altitude(70000.0, 101325.0, 288.15, excessive_lapse);

    assert_eq!(res_hyp.is_err(), res_baro.is_err());
    assert!(res_hyp.is_err() && res_baro.is_err());
}
