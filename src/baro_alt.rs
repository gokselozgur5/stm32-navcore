//! Hypsometric barometric altitude estimator.

/// Errors that can occur during barometric altitude estimation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaroAltError {
    NonFinite,
    InvalidPressure,
    InvalidReferencePressure,
    InvalidTemperature,
    InvalidLapseRate,
}
