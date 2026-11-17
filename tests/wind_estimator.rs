use core::f32::consts::PI;
use stm32_navcore::wind_estimator::*;

#[test]
fn test_wind_estimator_public_api_and_reexports() {
    let cfg = WindEstimatorConfig::default();
    let mut estimator = WindEstimator::new(cfg).unwrap();
    assert!(!estimator.is_initialized());
    assert_eq!(estimator.sample_count(), 0);
    assert_eq!(estimator.wind_vector(), [0.0, 0.0]);
    assert_eq!(estimator.wind_speed(), 0.0);
}
