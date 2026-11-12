use stm32_navcore::sensors::baro::BaroFilter;
use stm32_navcore::sensors::baro::{compensated_altitude_m, BaroError};
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

#[test]
fn isa_temperature_matches_plain_altitude() {
    let a = compensated_altitude_m(898.7, 1013.25, 8.5).unwrap();
    assert!((a - 1000.0).abs() < 5.0);
}

#[test]
fn warmer_air_gives_larger_altitude() {
    let cold = compensated_altitude_m(898.7, 1013.25, 15.0).unwrap();
    let warm = compensated_altitude_m(898.7, 1013.25, 35.0).unwrap();
    assert!(warm > cold);
}

#[test]
fn absolute_zero_is_rejected() {
    let e = compensated_altitude_m(898.7, 1013.25, -273.15);
    assert_eq!(e, Err(BaroError::Temperature));
}

#[test]
fn bad_pressure_is_reported() {
    assert!(matches!(
        compensated_altitude_m(-1.0, 1013.25, 15.0),
        Err(BaroError::Pressure(_))
    ));
}

#[test]
fn constant_input_converges() {
    let mut f = BaroFilter::new(0.2, 50.0, 3);
    let mut out = 0.0;
    for _ in 0..100 {
        out = f.update(10.0).unwrap();
    }
    assert!((out - 10.0).abs() < 1e-3);
}

#[test]
fn single_spike_is_rejected() {
    let mut f = BaroFilter::new(0.5, 5.0, 3);
    f.update(0.0);
    assert_eq!(f.update(100.0), None);
    assert!(f.update(0.0).unwrap().abs() < 1e-6);
}

#[test]
fn sustained_jump_is_accepted() {
    let mut f = BaroFilter::new(1.0, 5.0, 2);
    f.update(0.0);
    assert_eq!(f.update(100.0), None);
    assert_eq!(f.update(100.0), None);
    assert_eq!(f.update(100.0), Some(100.0));
}

#[test]
fn non_finite_is_rejected() {
    let mut f = BaroFilter::new(0.5, 5.0, 3);
    assert_eq!(f.update(f32::NAN), None);
}
