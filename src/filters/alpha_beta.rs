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

/// Alpha-beta tracker for scalar position and velocity tracking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlphaBetaFilter {
    alpha: f32,
    beta: f32,
    dt: f32,
    x: f32,
    v: f32,
    initialized: bool,
}

impl AlphaBetaFilter {
    /// Construct a new tracker with gains and sample interval.
    pub fn new(alpha: f32, beta: f32, dt: f32) -> Result<Self, AlphaBetaError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(AlphaBetaError::InvalidSampleInterval);
        }
        if !alpha.is_finite() || alpha <= 0.0 || alpha > 1.0 {
            return Err(AlphaBetaError::InvalidAlpha);
        }
        if !beta.is_finite() || beta <= 0.0 || beta > 2.0 * (2.0 - alpha) {
            return Err(AlphaBetaError::InvalidBeta);
        }
        Ok(Self {
            alpha,
            beta,
            dt,
            x: 0.0,
            v: 0.0,
            initialized: false,
        })
    }
}

