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
