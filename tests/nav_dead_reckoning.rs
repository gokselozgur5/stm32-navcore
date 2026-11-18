use stm32_navcore::nav::dead_reckoning::DeadReckoningState;

#[test]
fn test_dead_reckoning_state_init() {
    let state = DeadReckoningState::new(0.0, 0.0, 100.0, 10.0, 5.0, -1.0);
    assert_eq!(state.lat_rad, 0.0);
    assert_eq!(state.lon_rad, 0.0);
    assert_eq!(state.height_m, 100.0);
    assert_eq!(state.v_n, 10.0);
    assert_eq!(state.v_e, 5.0);
    assert_eq!(state.v_d, -1.0);
}

use stm32_navcore::nav::dead_reckoning::dead_reckoning_step;

#[test]
fn test_stationary_dr_invariance() {
    let state = DeadReckoningState::new(0.5, 0.5, 50.0, 0.0, 0.0, 0.0);
    let next = dead_reckoning_step(&state, 0.1).unwrap();
    assert_eq!(next.lat_rad, state.lat_rad);
    assert_eq!(next.lon_rad, state.lon_rad);
    assert_eq!(next.height_m, state.height_m);
}

#[test]
fn test_eastward_displacement_step() {
    let state = DeadReckoningState::new(0.0, 0.0, 0.0, 0.0, 100.0, 0.0);
    let next = dead_reckoning_step(&state, 1.0).unwrap();
    assert_eq!(next.lat_rad, 0.0);
    assert!(next.lon_rad > 0.0);
    assert_eq!(next.height_m, 0.0);
}

#[test]
fn test_northward_displacement_step() {
    let state = DeadReckoningState::new(0.0, 0.0, 0.0, 100.0, 0.0, 0.0);
    let next = dead_reckoning_step(&state, 1.0).unwrap();
    assert!(next.lat_rad > 0.0);
    assert_eq!(next.lon_rad, 0.0);
    assert_eq!(next.height_m, 0.0);
}

#[test]
fn test_vertical_displacement_step() {
    let state = DeadReckoningState::new(0.0, 0.0, 100.0, 0.0, 0.0, -10.0);
    let next = dead_reckoning_step(&state, 2.0).unwrap();
    assert_eq!(next.height_m, 120.0);
}

use stm32_navcore::nav::dead_reckoning::NavError;

#[test]
fn test_dr_validation_guards() {
    let state = DeadReckoningState::new(f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0);
    assert_eq!(dead_reckoning_step(&state, 1.0), Err(NavError::NonFinite));
    let valid = DeadReckoningState::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    assert_eq!(dead_reckoning_step(&valid, -1.0), Err(NavError::NonFinite));
}
