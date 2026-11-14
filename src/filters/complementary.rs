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
