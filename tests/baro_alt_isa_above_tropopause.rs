use stm32_navcore::baro_alt::{altitude_from_pressure_isa, pressure_at_altitude_isa};

// ICAO standard atmosphere (Doc 7488): the layer from 11 000 m to 20 000 m is isothermal at 216.65 K.
// p(15 000 m) = 22632.04 Pa * exp(-g0 * 4000 m / (R * 216.65 K)) = 12044.55 Pa.
// Either the correct value or an explicit error is acceptable.
#[test]
fn isa_conversion_is_correct_or_rejected_above_the_tropopause() {
    if let Ok(alt) = altitude_from_pressure_isa(12_044.55) {
        assert!((alt - 15_000.0).abs() < 5.0, "altitude = {alt} m");
    }
    if let Ok(p) = pressure_at_altitude_isa(15_000.0) {
        assert!((p - 12_044.55).abs() < 5.0, "pressure = {p} Pa");
    }
}
