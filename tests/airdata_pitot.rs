use stm32_navcore::airdata::{differential_pressure_to_ias, AirdataError, RHO0_ISA};

#[test]
fn pitot_differential_pressure_to_ias() {
    let dp = 0.5 * RHO0_ISA * 20.0 * 20.0;
    let ias = differential_pressure_to_ias(dp, RHO0_ISA).expect("ias computation");
    assert!((ias - 20.0).abs() < 1e-4);
}
