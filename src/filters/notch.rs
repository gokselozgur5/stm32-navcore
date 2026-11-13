//! Second-order notch filter for motor harmonic rejection.

use crate::filters::Biquad;

/// Second-order notch filter built upon a DF2T biquad section.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NotchFilter {
    biquad: Biquad,
}

use crate::filters::FilterError;

impl NotchFilter {
    /// Constructs a second-order notch filter.
    pub fn new(center_hz: f32, dt_s: f32, q: f32) -> Result<Self, FilterError> {
        if !center_hz.is_finite() || !dt_s.is_finite() || !q.is_finite() {
            return Err(FilterError::NonFinite);
        }
        if center_hz <= 0.0 { return Err(FilterError::InvalidCutoff); }
        if dt_s <= 0.0 { return Err(FilterError::InvalidDt); }
        if q <= 0.0 { return Err(FilterError::InvalidParameter); }
        if center_hz >= 0.5 / dt_s { return Err(FilterError::AboveNyquist); }

        let w0 = 2.0 * core::f32::consts::PI * center_hz * dt_s;
        let cos_w0 = libm::cosf(w0);
        let alpha = libm::sinf(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b0 = 1.0 / a0;
        let b1 = (-2.0 * cos_w0) / a0;
        let a2 = (1.0 - alpha) / a0;
        Ok(Self {
            biquad: Biquad::new(b0, b1, b0, b1, a2),
        })
    }
}

impl NotchFilter {
    /// Feeds one sample and returns the filtered output.
    pub fn update(&mut self, sample: f32) -> f32 {
        self.biquad.update(sample)
    }

    /// Resets internal filter states to zero.
    pub fn reset(&mut self) {
        self.biquad.reset();
    }

    /// Returns the underlying biquad filter section.
    pub const fn biquad(&self) -> &Biquad {
        &self.biquad
    }
}
