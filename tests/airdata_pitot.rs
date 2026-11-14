use stm32_navcore::airdata::{differential_pressure_to_ias, RHO0_ISA};

#[test]
fn pitot_differential_pressure_to_ias() {
    let dp = 0.5 * RHO0_ISA * 20.0 * 20.0;
    let ias = differential_pressure_to_ias(dp, RHO0_ISA).expect("ias computation");
    assert!((ias - 20.0).abs() < 1e-4);
}

#[test]
fn pitot_differential_pressure_invalid_density() {
    use stm32_navcore::airdata::AirdataError;
    assert_eq!(differential_pressure_to_ias(10.0, 0.0), Err(AirdataError::InvalidDensity));
    assert_eq!(differential_pressure_to_ias(10.0, -1.0), Err(AirdataError::InvalidDensity));
    assert_eq!(differential_pressure_to_ias(f32::NAN, 1.225), Err(AirdataError::NonFinite));
    assert_eq!(differential_pressure_to_ias(10.0, f32::NAN), Err(AirdataError::NonFinite));
}
