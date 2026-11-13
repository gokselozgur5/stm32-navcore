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
