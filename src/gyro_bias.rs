//! Complementary filter gyroscope bias drift estimator and tracker.

/// Errors returned by gyroscope bias tracking and compensation operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GyroBiasError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Gain parameter must be finite and within valid range.
    InvalidGain,
    /// Threshold or limit parameter must be finite and strictly positive.
    InvalidLimit,
    /// Input vector contains NaN or infinity.
    NonFinite,
}
