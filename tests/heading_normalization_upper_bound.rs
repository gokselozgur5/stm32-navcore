use stm32_navcore::geo::initial_bearing_deg;
use stm32_navcore::geo::wmm_declination::{
    normalize_heading_deg, normalize_heading_rad, true_to_magnetic_heading_deg,
};

#[test]
fn tiny_negative_angles_stay_below_the_upper_bound() {
    let two_pi = 2.0 * core::f32::consts::PI;
    let deg = normalize_heading_deg(-1.0e-6);
    let rad = normalize_heading_rad(-1.0e-8);
    let mag = true_to_magnetic_heading_deg(5.0, 5.000_000_5).unwrap();
    let brg = initial_bearing_deg(0.0, 0.0, 10.0, -1.0e-15).unwrap();
    assert!(deg < 360.0, "normalize_heading_deg(-1e-6) = {deg}");
    assert!(rad < two_pi, "normalize_heading_rad(-1e-8) = {rad}");
    assert!(mag < 360.0, "true_to_magnetic_heading_deg = {mag}");
    assert!(brg < 360.0, "initial_bearing_deg = {brg}");
}
