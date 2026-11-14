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
