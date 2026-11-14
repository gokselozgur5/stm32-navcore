use core::f32;

/// First-order IIR low-pass filter.
pub struct LowPassFilter {
    a: f32,
    b: f32,
}

impl LowPassFilter {
    /// Create a new first-order IIR low-pass filter with the given cutoff frequency and sample interval.
    pub fn new(cutoff_freq: f32, dt: f32) -> Self {
        let nyquist = 1.0 / (dt * 2.0);
        if cutoff_freq <= 0.0 || cutoff_freq >= nyquist {
            panic!("Cutoff frequency must be between 0 and Nyquist frequency.");
        }

        let a = cutoff_freq / (cutoff_freq + dt);
        let b = 1.0 - a;

        LowPassFilter { a, b }
    }

    /// Process the input signal.
    pub fn process(&mut self, input: f32) -> f32 {
