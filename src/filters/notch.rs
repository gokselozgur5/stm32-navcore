//! Second-order notch filter for motor harmonic rejection (NC-019).

use crate::filters::{BiquadCoefficients, BiquadFilter, LowPassError};

/// Second-order notch filter built upon a DF2T biquad section.
#[derive(Clone, Copy, Debug)]
pub struct NotchFilter {
    biquad: BiquadFilter,
}

impl NotchFilter {
    /// Constructs a second-order notch filter.
    pub fn new(center_hz: f32, dt_s: f32, q: f32) -> Result<Self, LowPassError> {
        if !center_hz.is_finite() || center_hz <= 0.0 || center_hz >= 0.5 / dt_s {
            return Err(LowPassError::InvalidCutoffFrequency);
        }
        if !dt_s.is_finite() || dt_s <= 0.0 || !q.is_finite() || q <= 0.0 {
            return Err(LowPassError::InvalidSampleInterval);
        }
        let w0 = 2.0 * core::f32::consts::PI * center_hz * dt_s;
        let cos_w0 = libm::cosf(w0);
        let alpha = libm::sinf(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b0 = 1.0 / a0;
        let b1 = (-2.0 * cos_w0) / a0;
        let a2 = (1.0 - alpha) / a0;
        let coeffs = BiquadCoefficients { b0, b1, b2: b0, a1: b1, a2 };
        Ok(Self { biquad: BiquadFilter::new(coeffs) })
    }
}
