use stm32_navcore::airdata::wind_estimator::{WindEstimator, WindEstimatorConfig};

#[test]
fn min_airspeed_above_wind_gate_is_a_valid_config() {
    // Fast airframe: skip samples below 30 m/s TAS, gate wind outliers above 25 m/s.
    let cfg = WindEstimatorConfig {
        time_constant_s: 10.0,
        min_airspeed_mps: 30.0,
        max_wind_speed_mps: 25.0,
    };
    assert_eq!(cfg.validate(), Ok(()));
    let mut est = WindEstimator::new(cfg).unwrap();
    let w = est.update([45.0, 0.0], 40.0, 0.0, 0.1).unwrap();
    assert!((w.wind_ne_mps[0] - 5.0).abs() < 1e-4);
}
