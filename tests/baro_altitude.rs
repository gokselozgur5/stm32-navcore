use stm32_navcore::sensors::baro::{BaroMeasurement, ISA_SEA_LEVEL_HPA};

#[test]
fn sea_level_pressure_is_zero_altitude() {
    let m = BaroMeasurement::from_pressure(ISA_SEA_LEVEL_HPA, ISA_SEA_LEVEL_HPA, 1.0).unwrap();
    assert!(m.altitude_m.abs() < 1e-3);
    assert_eq!(m.variance_m2, 1.0);
}

#[test]
fn one_km_altitude_near_898_7_hpa() {
    let m = BaroMeasurement::from_pressure(898.7, ISA_SEA_LEVEL_HPA, 1.0).unwrap();
    assert!((m.altitude_m - 1000.0).abs() < 5.0);
}

#[test]
fn invalid_pressure_is_rejected() {
    assert!(BaroMeasurement::from_pressure(0.0, ISA_SEA_LEVEL_HPA, 1.0).is_err());
    assert!(BaroMeasurement::from_pressure(f32::NAN, ISA_SEA_LEVEL_HPA, 1.0).is_err());
}
