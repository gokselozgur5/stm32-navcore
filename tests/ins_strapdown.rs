use stm32_navcore::geo::earth_rate::principal_radii_of_curvature;
use stm32_navcore::ins_strapdown::meridian_radius;

#[test]
fn test_ins_strapdown_uses_geo_helpers() {
    let lat = core::f64::consts::FRAC_PI_4;
    let (geo_rm, _) = principal_radii_of_curvature(lat).expect("geo radii");
    let ins_rm = meridian_radius(lat).expect("ins radii");
    assert_eq!(ins_rm, geo_rm);
}
