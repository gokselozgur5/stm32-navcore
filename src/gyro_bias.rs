//! Complementary filter gyroscope bias drift estimator and tracker.

/// Errors returned by gyroscope bias tracking and compensation operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GyroBiasError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Gain parameter must be finite and within valid range.
    InvalidGain,
    /// Threshold or limit parameter must be finite and strictly positive.
    InvalidLimit,
    /// Input vector contains NaN or infinity.
    NonFinite,
}

/// Configuration parameters for gyroscope bias tracking.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroBiasConfig {
    /// Integral gain for tracking attitude/angular rate error.
    pub ki: f32,
    /// Complementary filter gain for stationary zero-rate updates.
    pub stationary_learning_rate: f32,
    /// Maximum absolute bias per axis in rad/s.
    pub max_bias_rad_s: f32,
    /// Maximum rate of bias change in rad/s^2.
    pub max_drift_rate_rad_s2: f32,
    /// Angular rate magnitude threshold for stationary detection in rad/s.
    pub stationary_threshold_rad_s: f32,
}

impl GyroBiasConfig {
    /// Validate configuration invariants.
    pub fn validate(&self) -> Result<(), GyroBiasError> {
        if !self.ki.is_finite() || self.ki < 0.0 {
            return Err(GyroBiasError::InvalidGain);
        }
        if !self.stationary_learning_rate.is_finite()
            || self.stationary_learning_rate <= 0.0
            || self.stationary_learning_rate > 1.0
        {
            return Err(GyroBiasError::InvalidGain);
        }
        if !self.max_bias_rad_s.is_finite() || self.max_bias_rad_s <= 0.0 {
            return Err(GyroBiasError::InvalidLimit);
        }
        if !self.max_drift_rate_rad_s2.is_finite() || self.max_drift_rate_rad_s2 <= 0.0 {
            return Err(GyroBiasError::InvalidLimit);
        }
        if !self.stationary_threshold_rad_s.is_finite() || self.stationary_threshold_rad_s <= 0.0 {
            return Err(GyroBiasError::InvalidLimit);
        }
        Ok(())
    }
}

impl Default for GyroBiasConfig {
    fn default() -> Self {
        Self {
            ki: 0.01,
            stationary_learning_rate: 0.05,
            max_bias_rad_s: 0.1,
            max_drift_rate_rad_s2: 0.005,
            stationary_threshold_rad_s: 0.035,
        }
    }
}

/// Correct raw gyroscope measurement with estimated bias.
pub fn correct_gyro_sample(
    raw: [f32; 3],
    bias: [f32; 3],
) -> Result<[f32; 3], GyroBiasError> {
    for i in 0..3 {
        if !raw[i].is_finite() || !bias[i].is_finite() {
            return Err(GyroBiasError::NonFinite);
        }
    }
    Ok([raw[0] - bias[0], raw[1] - bias[1], raw[2] - bias[2]])
}
