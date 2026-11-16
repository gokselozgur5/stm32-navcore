use stm32_navcore::gravity::{
    somigliana_surface, GAMMA_E, GAMMA_P,
};

#[test]
fn equator_and_pole_surface_gravity() {
    let g_eq = somigliana_surface(0.0).expect("equator gravity");
    assert!((g_eq - GAMMA_E).abs() < 1e-5);

    let half_pi = core::f32::consts::FRAC_PI_2;
    let g_north = somigliana_surface(half_pi).expect("north pole gravity");
    assert!((g_north - GAMMA_P).abs() < 1e-4);

    let g_south = somigliana_surface(-half_pi).expect("south pole gravity");
    assert!((g_south - GAMMA_P).abs() < 1e-4);
}

use stm32_navcore::gravity::{elevation_correction, normal_gravity};

#[test]
fn elevation_correction_decreases_gravity() {
    let lat_45 = core::f32::consts::FRAC_PI_4;
    let g_surf = normal_gravity(lat_45, 0.0).expect("surface gravity");
    let g_1000m = normal_gravity(lat_45, 1000.0).expect("1000m gravity");
    assert!(g_1000m < g_surf);

    let delta = g_surf - g_1000m;
    assert!((delta - 0.00308).abs() < 1e-4);

    let corr = elevation_correction(lat_45, 1000.0).expect("correction");
    assert!((corr - (-0.00308)).abs() < 1e-4);
}

use stm32_navcore::gravity::{gravity_vector_ned, GravityError};

#[test]
fn ned_vector_and_input_validation() {
    let vec = gravity_vector_ned(0.0, 0.0).expect("ned vector");
    assert_eq!(vec[0], 0.0);
    assert_eq!(vec[1], 0.0);
    assert!((vec[2] - 9.780_325).abs() < 1e-4);

    assert_eq!(somigliana_surface(f32::NAN), Err(GravityError::NonFinite));
    assert_eq!(somigliana_surface(2.0), Err(GravityError::InvalidLatitude));
    assert_eq!(normal_gravity(0.0, 200_000.0), Err(GravityError::InvalidAltitude));
}
