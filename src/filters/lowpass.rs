//! First-order IIR low-pass filter.

/// Errors from filter parameter validation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FilterError {
    /// Cutoff frequency is non-positive or not finite.
    InvalidCutoff,
    /// Sample interval is non-positive or not finite.
    InvalidDt,
    /// Cutoff frequency is at or above the Nyquist frequency.
    AboveNyquist,
    /// Value is NaN or infinite.
    NonFinite,
}

/// First-order IIR low-pass filter.
#[derive(Clone, Copy, Debug)]
pub struct LowPassFilter {
    alpha: f32,
    state: Option<f32>,
}
