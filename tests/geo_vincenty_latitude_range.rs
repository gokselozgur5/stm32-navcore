use stm32_navcore::geo::vincenty;

#[test]
fn inverse_rejects_latitude_beyond_the_pole() {
    let deg = core::f64::consts::PI / 180.0;
    let out = vincenty::inverse(100.0 * deg, 0.0, 10.0 * deg, 1.0 * deg);
    assert!(out.is_none(), "{out:?}");
}
