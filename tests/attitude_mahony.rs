use stm32_navcore::attitude::{AttitudeError, Mahony};
use stm32_navcore::math::Quaternion;

#[test]
fn test_invalid_dt_and_non_finite_inputs_rejected() {
    let mut filter = Mahony::default();
    let g = [0.0, 0.0, 0.0];
    let a = [0.0, 0.0, -9.81];

    assert_eq!(filter.update(g, a, 0.0), Err(AttitudeError::InvalidInput));
    assert_eq!(filter.update(g, a, -0.01), Err(AttitudeError::InvalidInput));
    assert_eq!(
        filter.update(g, a, f32::NAN),
        Err(AttitudeError::InvalidInput)
    );
    assert_eq!(
        filter.update([f32::NAN, 0.0, 0.0], a, 0.01),
        Err(AttitudeError::NonFinite)
    );
    assert_eq!(
        filter.update(g, [0.0, f32::INFINITY, 0.0], 0.01),
        Err(AttitudeError::NonFinite)
    );
}
