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
