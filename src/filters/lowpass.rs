//! First-order discrete IIR low-pass filter (NC-017).

/// Error returned when low-pass filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LowPassError {
    /// Cutoff frequency must be positive and below Nyquist.
    InvalidCutoffFrequency,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
}

/// First-order single-pole discrete IIR low-pass filter.
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter {
    alpha: f32,
    prev_output: f32,
    initialized: bool,
}
