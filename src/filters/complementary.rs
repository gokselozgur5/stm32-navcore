//! 2-axis pitch/roll complementary filter in no-std.

/// Error returned when complementary filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComplementaryError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Alpha weight must be in (0, 1).
    InvalidAlpha,
}
