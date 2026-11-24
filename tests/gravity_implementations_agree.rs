use stm32_navcore::geo::{gravity, gravity_somigliana, gravity_wgs84};

// At 10 000 km altitude gravity is about GM / r^2 = 1.49 m/s^2.
#[test]
fn all_normal_gravity_implementations_agree_on_altitude_limits() {
    let a = gravity::normal_gravity(0.5, 1.0e7);
    let b = gravity_wgs84::normal_gravity_elevation(0.5, 1.0e7);
    let c = gravity_somigliana::normal_gravity_f64(0.5, 1.0e7);
    assert_eq!(a.is_ok(), b.is_ok());
    assert_eq!(a.is_ok(), c.is_ok());
}
