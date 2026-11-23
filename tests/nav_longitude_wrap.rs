use core::f64::consts::PI;
use stm32_navcore::ins_strapdown::{InsStrapdownIntegrator, StrapdownState};
use stm32_navcore::nav::{dead_reckoning_step, DeadReckoningState};

#[test]
fn dead_reckoning_wraps_longitude_at_the_antimeridian() {
    let east = DeadReckoningState::new(0.5, core::f64::consts::PI, 100.0, 0.0, 300.0, 0.0);
    let next = dead_reckoning_step(&east, 10.0).unwrap();
    assert!(next.lon_rad.abs() <= PI, "lon_rad {}", next.lon_rad);
}

#[test]
fn strapdown_wraps_longitude_at_the_antimeridian() {
    let mut state = StrapdownState::new(0.5, core::f64::consts::PI, 100.0);
    state.vel_ned = [0.0, 300.0, 0.0];
    let mut ins = InsStrapdownIntegrator::new(state);
    for _ in 0..100 {
        ins.step([0.0, 0.0, -9.8], [0.0; 3], 0.1).unwrap();
    }
    assert!(
        ins.state().lon_rad.abs() <= PI,
        "lon_rad {}",
        ins.state().lon_rad
    );
}
