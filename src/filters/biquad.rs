//! Direct Form II Transposed (DF2T) biquad filter section (NC-018).

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
}
