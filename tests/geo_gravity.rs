use stm32_navcore::geo::{normal_gravity, normal_gravity_surface};

#[test]
fn equator_and_pole_values_match_acceptance_criteria() {
    let g_equator = normal_gravity_surface(0.0).expect("equator gravity");
    assert!((g_equator - 9.7803).abs() < 1e-4);

    let half_pi = core::f64::consts::FRAC_PI_2;
    let g_pole = normal_gravity_surface(half_pi).expect("north pole gravity");
    assert!((g_pole - 9.8322).abs() < 1e-4);

    let g_south_pole = normal_gravity_surface(-half_pi).expect("south pole gravity");
    assert!((g_south_pole - 9.8322).abs() < 1e-4);
}

#[test]
fn free_air_correction_decreases_gravity_with_altitude() {
    let lat_45 = core::f64::consts::FRAC_PI_4;
    let g_surface = normal_gravity(lat_45, 0.0).expect("surface gravity");
    let g_1000m = normal_gravity(lat_45, 1000.0).expect("1000m gravity");
    assert!(g_1000m < g_surface);

    let delta_g = g_surface - g_1000m;
    assert!((delta_g - 0.00308).abs() < 1e-4);
}

#[test]
fn invalid_and_non_finite_inputs_are_rejected() {
    use stm32_navcore::geo::GeoError;

    assert_eq!(normal_gravity_surface(f64::NAN), Err(GeoError::NonFinite));
    assert_eq!(normal_gravity_surface(2.0), Err(GeoError::InvalidLatitude));
    assert_eq!(normal_gravity_surface(-2.0), Err(GeoError::InvalidLatitude));

    assert_eq!(normal_gravity(0.0, f64::NAN), Err(GeoError::NonFinite));
}
