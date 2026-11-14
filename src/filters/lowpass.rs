//! First-order discrete IIR low-pass filter (NC-017).

/// Error returned when low-pass filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LowPassError {
    /// Cutoff frequency must be positive and below Nyquist.
    InvalidCutoffFrequency,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
}

/// First-order single-pole discrete IIR low-pass filter.
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter {
    alpha: f32,
    prev_output: f32,
    initialized: bool,
}

impl LowPassFilter {
    /// Construct a new low-pass filter from cutoff frequency and sample interval.
    pub fn new(cutoff_hz: f32, dt: f32) -> Result<Self, LowPassError> {
        if !cutoff_hz.is_finite() || cutoff_hz <= 0.0 {
            return Err(LowPassError::InvalidCutoffFrequency);
        }
        if !dt.is_finite() || dt <= 0.0 {
            return Err(LowPassError::InvalidSampleInterval);
        }
        let nyquist = 0.5 / dt;
        if cutoff_hz >= nyquist {
            return Err(LowPassError::InvalidCutoffFrequency);
        }
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        let alpha = dt / (rc + dt);
        Ok(Self {
            alpha,
            prev_output: 0.0,
            initialized: false,
        })
    }
}

impl LowPassFilter {
    /// Return the smoothing factor alpha.
    pub const fn alpha(&self) -> f32 {
        self.alpha
    }

    /// Reset internal state to uninitialized.
    pub fn reset(&mut self) {
        self.prev_output = 0.0;
        self.initialized = false;
    }

    /// Update filter with a new sample and return filtered output.
    pub fn update(&mut self, sample: f32) -> f32 {
        if !self.initialized {
            self.prev_output = sample;
            self.initialized = true;
            sample
        } else {
            let out = self.prev_output + self.alpha * (sample - self.prev_output);
            self.prev_output = out;
            out
        }
    }
}
