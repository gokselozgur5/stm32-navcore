//! Slew-rate limiter and symmetric deadband (NC-022).

/// Error type for slew rate limiter configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlewError {
    /// Limit or sample interval is non-positive or non-finite.
    InvalidParameter,
}

/// Slew-rate limiter bounding output change per step.
#[derive(Debug, Clone, Copy)]
pub struct SlewRateLimiter {
    max_rate: f32,
    dt: f32,
    prev_output: f32,
    initialized: bool,
}

impl SlewRateLimiter {
    /// Construct a new slew-rate limiter.
    pub fn new(max_rate: f32, dt: f32) -> Result<Self, SlewError> {
        if !max_rate.is_finite() || max_rate <= 0.0 || !dt.is_finite() || dt <= 0.0 {
            return Err(SlewError::InvalidParameter);
        }
        Ok(Self {
            max_rate,
            dt,
            prev_output: 0.0,
            initialized: false,
        })
    }
}
