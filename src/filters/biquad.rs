//! Direct Form II Transposed (DF2T) biquad filter section (NC-018).

/// Error returned when biquad filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BiquadError {
    /// Cutoff frequency must be positive and below Nyquist.
    InvalidCutoffFrequency,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
    /// Q factor must be positive and finite.
    InvalidQ,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BiquadCoefficients {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct BiquadFilter {
    coeffs: BiquadCoefficients,
    s1: f32,
    s2: f32,
}

impl BiquadCoefficients {
    pub fn try_lowpass_rbj(cutoff_hz: f32, sample_interval_s: f32, q: f32) -> Result<Self, BiquadError> {
        if !sample_interval_s.is_finite() || sample_interval_s <= 0.0 {
            return Err(BiquadError::InvalidSampleInterval);
        }
        if !cutoff_hz.is_finite() || cutoff_hz <= 0.0 || cutoff_hz >= 0.5 / sample_interval_s {
            return Err(BiquadError::InvalidCutoffFrequency);
        }
        if !q.is_finite() || q <= 0.0 {
            return Err(BiquadError::InvalidQ);
        }
        Ok(Self::lowpass_rbj(cutoff_hz, sample_interval_s, q))
    }

    pub fn lowpass_rbj(cutoff_hz: f32, sample_interval_s: f32, q: f32) -> Self {
        let fs = 1.0 / sample_interval_s;
        let w0 = 2.0 * core::f32::consts::PI * cutoff_hz / fs;
        let cos_w0 = libm::cosf(w0);
        let alpha = libm::sinf(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b0 = ((1.0 - cos_w0) / 2.0) / a0;
        let b1 = (1.0 - cos_w0) / a0;
        let b2 = ((1.0 - cos_w0) / 2.0) / a0;
        let a1 = (-2.0 * cos_w0) / a0;
        let a2 = (1.0 - alpha) / a0;
        Self { b0, b1, b2, a1, a2 }
    }

    /// Checks whether the filter is BIBO stable (|a2| < 1 and |a1| < 1 + a2).
    pub fn is_stable(&self) -> bool {
        self.a2.abs() < 1.0 && self.a1.abs() < 1.0 + self.a2
    }
}

impl BiquadFilter {
    pub const fn new(coeffs: BiquadCoefficients) -> Self {
        Self {
            coeffs,
            s1: 0.0,
            s2: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }

    pub fn update(&mut self, x: f32) -> f32 {
        let y = self.coeffs.b0 * x + self.s1;
        self.s1 = self.coeffs.b1 * x - self.coeffs.a1 * y + self.s2;
        self.s2 = self.coeffs.b2 * x - self.coeffs.a2 * y;
        y
    }
}

impl BiquadFilter {
    pub fn coefficients(&self) -> &BiquadCoefficients {
        &self.coeffs
    }

    pub fn set_coefficients(&mut self, coeffs: BiquadCoefficients) {
        self.coeffs = coeffs;
    }

    /// Returns the internal filter delay states (s1, s2).
    pub const fn state(&self) -> (f32, f32) {
        (self.s1, self.s2)
    }
}
