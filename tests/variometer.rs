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
