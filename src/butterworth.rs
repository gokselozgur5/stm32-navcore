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

/// Biquad filter transfer function coefficients for 2nd-order Butterworth.
///
/// H(z) = (b0 + b1 * z^-1 + b2 * z^-2) / (1 + a1 * z^-1 + a2 * z^-2)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ButterworthCoefficients {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
}
