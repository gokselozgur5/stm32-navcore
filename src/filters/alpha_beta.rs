//! Alpha-beta tracker for scalar position and velocity (NC-021).

/// Error returned when tracker parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlphaBetaError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Alpha smoothing factor must be in (0, 1].
    InvalidAlpha,
    /// Beta velocity factor must be non-negative and satisfy stability bounds.
    InvalidBeta,
}
