//! Barometric altitude hypsometric equation and temperature lapse compensator.

/// Errors occurring in hypsometric and lapse calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HypsometricError {
    /// Non-finite numeric input (NaN or Inf).
    NonFinite,
    /// Non-positive atmospheric pressure value.
    InvalidPressure,
    /// Absolute temperature below zero Kelvin.
    InvalidTemperature,
    /// Negative or excessive lapse rate.
    InvalidLapseRate,
    /// Division by zero or indeterminate ratio.
    ZeroDivision,
}
