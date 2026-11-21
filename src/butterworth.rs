//! Second-order low-pass IIR Butterworth filter with bilinear transform.

use crate::filters::{BiquadCoefficients, BiquadError, BiquadFilter};

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

impl From<BiquadError> for ButterworthError {
    fn from(err: BiquadError) -> Self {
        match err {
            BiquadError::InvalidCutoffFrequency | BiquadError::InvalidQ => {
                Self::InvalidCutoffFrequency
            }
            BiquadError::InvalidSampleInterval => Self::InvalidSampleInterval,
            BiquadError::NonFinite => Self::NonFinite,
            BiquadError::BufferLengthMismatch => Self::BufferLengthMismatch,
        }
    }
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
        let biquad = BiquadCoefficients::butterworth(cutoff_hz, sample_interval_s)?;
        Ok(Self::from_biquad(biquad))
    }

    /// Converts from generic BiquadCoefficients.
    pub const fn from_biquad(b: BiquadCoefficients) -> Self {
        Self {
            b0: b.b0,
            b1: b.b1,
            b2: b.b2,
            a1: b.a1,
            a2: b.a2,
        }
    }

    /// Converts to generic BiquadCoefficients.
    pub const fn to_biquad(&self) -> BiquadCoefficients {
        BiquadCoefficients {
            b0: self.b0,
            b1: self.b1,
            b2: self.b2,
            a1: self.a1,
            a2: self.a2,
        }
    }

    /// Verifies Bounded-Input Bounded-Output (BIBO) stability.
    pub fn is_stable(&self) -> bool {
        self.to_biquad().is_stable()
    }
}

/// Second-order low-pass Butterworth IIR filter (DF2T).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ButterworthFilter {
    biquad: BiquadFilter,
    coeffs: ButterworthCoefficients,
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
            biquad: BiquadFilter::new(coeffs.to_biquad()),
            coeffs,
        }
    }

    /// Resets filter delay states to zero.
    pub fn reset(&mut self) {
        self.biquad.reset();
    }

    /// Direct Form II Transposed update step.
    pub fn update(&mut self, x: f32) -> f32 {
        self.biquad.update(x)
    }

    /// Returns current filter internal state (s1, s2).
    pub const fn state(&self) -> (f32, f32) {
        self.biquad.state()
    }

    /// Sets internal delay states explicitly.
    pub fn set_state(&mut self, s1: f32, s2: f32) {
        self.biquad.set_state(s1, s2);
    }

    /// Validates sample is finite before executing filter step.
    pub fn try_update(&mut self, x: f32) -> Result<f32, ButterworthError> {
        self.biquad.try_update(x).map_err(Into::into)
    }

    /// Returns a reference to the active coefficients.
    pub fn coefficients(&self) -> &ButterworthCoefficients {
        &self.coeffs
    }

    /// Dynamically retunes filter coefficients without resetting states.
    pub fn retune(
        &mut self,
        cutoff_hz: f32,
        sample_interval_s: f32,
    ) -> Result<(), ButterworthError> {
        let coeffs = ButterworthCoefficients::try_new(cutoff_hz, sample_interval_s)?;
        self.set_coefficients(coeffs);
        Ok(())
    }

    /// Replaces coefficients explicitly.
    pub fn set_coefficients(&mut self, coeffs: ButterworthCoefficients) {
        self.coeffs = coeffs;
        self.biquad.set_coefficients(coeffs.to_biquad());
    }

    /// Filters an array of input samples into an equal-sized output slice.
    pub fn batch_filter(
        &mut self,
        input: &[f32],
        output: &mut [f32],
    ) -> Result<(), ButterworthError> {
        self.biquad.batch_filter(input, output).map_err(Into::into)
    }
}
