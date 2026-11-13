//! First-order IIR low-pass filter.

/// Errors from filter parameter validation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FilterError {
    /// Cutoff frequency is non-positive or not finite.
    InvalidCutoff,
    /// Sample interval is non-positive or not finite.
    InvalidDt,
    /// Cutoff frequency is at or above the Nyquist frequency.
    AboveNyquist,
    /// Value is NaN or infinite.
    NonFinite,
}

/// First-order IIR low-pass filter.
#[derive(Clone, Copy, Debug)]
pub struct LowPassFilter {
    alpha: f32,
    state: Option<f32>,
}

impl LowPassFilter {
    /// Constructs a low-pass filter from cutoff frequency and sample interval.
    pub fn from_cutoff(cutoff_hz: f32, dt_s: f32) -> Result<Self, FilterError> {
        if !cutoff_hz.is_finite() || !dt_s.is_finite() {
            return Err(FilterError::NonFinite);
        }
        if cutoff_hz <= 0.0 {
            return Err(FilterError::InvalidCutoff);
        }
        if dt_s <= 0.0 {
            return Err(FilterError::InvalidDt);
        }
        if cutoff_hz >= 0.5 / dt_s {
            return Err(FilterError::AboveNyquist);
        }
        let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        let alpha = 1.0 - libm::expf(-dt_s / tau);
        Ok(Self { alpha, state: None })
    }

    /// Smoothing coefficient alpha in (0, 1).
    pub const fn alpha(&self) -> f32 {
        self.alpha
    }
}
