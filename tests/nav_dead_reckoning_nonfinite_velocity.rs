use stm32_navcore::nav::{dead_reckoning_step, DeadReckoningState, NavError};

#[test]
fn nan_velocity_is_rejected() {
    let state = DeadReckoningState::new(0.5, 0.5, 100.0, f64::NAN, 0.0, 0.0);
    assert_eq!(dead_reckoning_step(&state, 1.0), Err(NavError::NonFinite));
}

#[test]
fn inf_velocity_is_rejected() {
    let state = DeadReckoningState::new(0.5, 0.5, 100.0, 0.0, f64::INFINITY, 0.0);
    assert_eq!(dead_reckoning_step(&state, 1.0), Err(NavError::NonFinite));
}
