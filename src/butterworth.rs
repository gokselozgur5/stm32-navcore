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

impl ButterworthCoefficients {
    /// Computes 2nd-order low-pass Butterworth coefficients via bilinear transform.
    pub fn try_new(cutoff_hz: f32, sample_interval_s: f32) -> Result<Self, ButterworthError> {
        if !sample_interval_s.is_finite() || sample_interval_s <= 0.0 {
            return Err(ButterworthError::InvalidSampleInterval);
        }
        let nyquist = 0.5 / sample_interval_s;
        if !cutoff_hz.is_finite() || cutoff_hz <= 0.0 || cutoff_hz >= nyquist {
            return Err(ButterworthError::InvalidCutoffFrequency);
        }
        let k = tanf(PI * cutoff_hz * sample_interval_s);
        let k2 = k * k;
        let sqrt2_k = 1.41421356f32 * k;
        let a0 = 1.0 + sqrt2_k + k2;
        let inv_a0 = 1.0 / a0;

        let b0 = k2 * inv_a0;
        let b1 = 2.0 * b0;
        let b2 = b0;
        let a1 = 2.0 * (k2 - 1.0) * inv_a0;
        let a2 = (1.0 - sqrt2_k + k2) * inv_a0;

        Ok(Self { b0, b1, b2, a1, a2 })
    }
}

impl ButterworthCoefficients {
    /// Verifies Bounded-Input Bounded-Output (BIBO) stability.
    ///
    /// Requires poles to lie strictly inside the unit circle:
    /// |a2| < 1.0 and |a1| < 1.0 + a2.
    pub fn is_stable(&self) -> bool {
        self.a2.abs() < 1.0 && self.a1.abs() < (1.0 + self.a2)
    }
}

/// Second-order low-pass Butterworth IIR filter (DF2T).
#[derive(Debug, Clone, Copy)]
pub struct ButterworthFilter {
    coeffs: ButterworthCoefficients,
    s1: f32,
    s2: f32,
}

impl ButterworthFilter {
    /// Creates a filter with specified cutoff and sample interval.
    pub fn try_new(cutoff_hz: f32, sample_interval_s: f32) -> Result<Self, ButterworthError> {
        let coeffs = ButterworthCoefficients::try_new(cutoff_hz, sample_interval_s)?;
        Ok(Self::from_coeffs(coeffs))
    }

    /// Creates a filter from precomputed coefficients.
    pub const fn from_coeffs(coeffs: ButterworthCoefficients) -> Self {
        Self {
            coeffs,
            s1: 0.0,
            s2: 0.0,
        }
    }
}
