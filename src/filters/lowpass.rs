//! First-order discrete IIR low-pass filter (NC-017).

/// Error returned when low-pass filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LowPassError {
    /// Cutoff frequency must be positive and below Nyquist.
    InvalidCutoffFrequency,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
}
