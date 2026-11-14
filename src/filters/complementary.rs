//! 2-axis pitch/roll complementary filter in no-std.

/// Error returned when complementary filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComplementaryError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Alpha weight must be in (0, 1).
    InvalidAlpha,
}

/// 2-axis complementary filter for attitude estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComplementaryFilter {
    alpha: f32,
    dt: f32,
    roll: f32,
    pitch: f32,
}

impl ComplementaryFilter {
    /// Creates a new complementary filter with given alpha and dt.
    pub fn new(alpha: f32, dt: f32) -> Result<Self, ComplementaryError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(ComplementaryError::InvalidSampleInterval);
        }
        if !alpha.is_finite() || alpha <= 0.0 || alpha >= 1.0 {
            return Err(ComplementaryError::InvalidAlpha);
        }
        Ok(Self { alpha, dt, roll: 0.0, pitch: 0.0 })
    }
}
