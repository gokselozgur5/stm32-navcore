//! Second-order low-pass IIR Butterworth filter with bilinear transform.

use core::f32::consts::PI;
use libm::tanf;

/// Errors returned by Butterworth filter operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButterworthError {
    /// Cutoff frequency must be positive and below Nyquist (fs / 2).
    InvalidCutoffFrequency,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
    /// Non-finite sample, state, or parameter encountered.
    NonFinite,
    /// Buffer length mismatch in batch processing.
    BufferLengthMismatch,
}
