//! Direct Form II Transposed (DF2T) biquad filter.

/// Direct Form II Transposed biquad filter section.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    s1: f32,
    s2: f32,
}

impl Biquad {
    /// Creates a biquad from normalized coefficients.
    pub const fn new(b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) -> Self {
        Self { b0, b1, b2, a1, a2, s1: 0.0, s2: 0.0 }
    }

    /// Resets internal filter states to zero.
    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }

    /// Returns internal DF2T states (s1, s2).
    pub const fn states(&self) -> (f32, f32) {
        (self.s1, self.s2)
    }

    /// Returns normalized coefficients (b0, b1, b2, a1, a2).
    pub const fn coefficients(&self) -> (f32, f32, f32, f32, f32) {
        (self.b0, self.b1, self.b2, self.a1, self.a2)
    }
}

use crate::filters::FilterError;

impl Biquad {
    /// Robert Bristow-Johnson (RBJ) low-pass filter design.
    pub fn lowpass_rbj(cutoff_hz: f32, dt_s: f32, q: f32) -> Result<Self, FilterError> {
        if !cutoff_hz.is_finite() || !dt_s.is_finite() || !q.is_finite() {
            return Err(FilterError::NonFinite);
        }
        if cutoff_hz <= 0.0 { return Err(FilterError::InvalidCutoff); }
        if dt_s <= 0.0 { return Err(FilterError::InvalidDt); }
        if q <= 0.0 { return Err(FilterError::InvalidParameter); }
        if cutoff_hz >= 0.5 / dt_s { return Err(FilterError::AboveNyquist); }
        let w0 = 2.0 * core::f32::consts::PI * cutoff_hz * dt_s;
        let cos_w0 = libm::cosf(w0);
        let alpha = libm::sinf(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b0 = ((1.0 - cos_w0) * 0.5) / a0;
        let b1 = (1.0 - cos_w0) / a0;
        let a1 = (-2.0 * cos_w0) / a0;
        let a2 = (1.0 - alpha) / a0;
        Ok(Self::new(b0, b1, b0, a1, a2))
    }
}

impl Biquad {
    /// Feeds one sample and returns the filtered output using DF2T.
    pub fn update(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.s1;
        self.s1 = self.b1 * x - self.a1 * y + self.s2;
        self.s2 = self.b2 * x - self.a2 * y;
        y
    }
}
