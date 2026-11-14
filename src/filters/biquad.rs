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
