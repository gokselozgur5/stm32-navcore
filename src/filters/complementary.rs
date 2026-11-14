//! Two-axis pitch/roll complementary filter (no-std).

/// Error for invalid complementary filter parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComplementaryError {
    /// Time constant tau must be positive and finite.
    InvalidTimeConstant,
    /// Sample interval dt must be positive and finite.
    InvalidSampleInterval,
}

/// Two-axis pitch and roll complementary filter.
#[derive(Debug, Clone, Copy)]
pub struct ComplementaryFilter {
    alpha: f32,
    pitch: f32,
    roll: f32,
    initialized: bool,
}
