//! True airspeed (TAS) estimation from pitot differential pressure and indicated airspeed (IAS).

/// Errors in pitot and airspeed calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PitotError {
    /// Input value is NaN or infinite.
    NonFinite,
    /// Static pressure is non-positive or unphysical.
    InvalidPressure,
    /// Absolute temperature is non-positive or below zero Kelvin.
    InvalidTemperature,
    /// Differential pressure is negative.
    NegativeDifferentialPressure,
    /// Indicated airspeed is negative.
    NegativeAirspeed,
    /// Calculated density is non-positive or non-finite.
    InvalidDensity,
}
