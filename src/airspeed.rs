//! Pitot-static indicated airspeed (IAS) and true airspeed (TAS) calculator.

/// Errors occurring in airspeed and atmospheric computations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirspeedError {
    /// Non-finite input parameter.
    NonFinite,
    /// Absolute or static pressure is non-positive.
    InvalidPressure,
    /// Absolute temperature is non-positive.
    InvalidTemperature,
    /// Differential pressure is negative.
    NegativeDifferentialPressure,
    /// Calculated or supplied density is non-positive.
    InvalidDensity,
    /// Airspeed is negative.
    NegativeAirspeed,
    /// Subsonic model exceeded (Mach >= 1.0).
    SupersonicFlow,
}
