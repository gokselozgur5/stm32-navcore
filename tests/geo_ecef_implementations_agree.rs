use stm32_navcore::geo::ecef_ned::{ecef_to_geodetic, EcefPos};
use stm32_navcore::geo::{ecef_to_lla, Ecef};

// Both functions implement the same Bowring conversion and must agree.
#[test]
fn both_ecef_to_geodetic_implementations_agree() {
    let lla = ecef_to_lla(&Ecef::new(1000.0, 0.0, 0.0)).map(|p| (p.lat_rad, p.alt_m));
    let geo = ecef_to_geodetic(&EcefPos::new(1000.0, 0.0, 0.0)).map(|p| (p.lat_rad, p.alt_m));
    assert_eq!(lla.ok(), geo.ok());
}
