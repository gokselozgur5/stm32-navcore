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

/// Clamp bias vector components to allowable maximum absolute value.
pub fn clamp_bias_vector(
    bias: [f32; 3],
    max_bias: f32,
) -> Result<[f32; 3], GyroBiasError> {
    if !max_bias.is_finite() || max_bias <= 0.0 {
        return Err(GyroBiasError::InvalidLimit);
    }
    let mut clamped = [0.0f32; 3];
    for i in 0..3 {
        if !bias[i].is_finite() {
            return Err(GyroBiasError::NonFinite);
        }
        if bias[i] > max_bias {
            clamped[i] = max_bias;
        } else if bias[i] < -max_bias {
            clamped[i] = -max_bias;
        } else {
            clamped[i] = bias[i];
        }
    }
    Ok(clamped)
}

/// Apply slew-rate limit to prevent discontinuous jumps in estimated bias.
pub fn apply_bias_slew_limit(
    new_bias: [f32; 3],
    prev_bias: [f32; 3],
    max_drift_rate: f32,
    dt_s: f32,
) -> Result<[f32; 3], GyroBiasError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(GyroBiasError::InvalidSampleInterval);
    }
    if !max_drift_rate.is_finite() || max_drift_rate <= 0.0 {
        return Err(GyroBiasError::InvalidLimit);
    }
    let max_delta = max_drift_rate * dt_s;
    let mut limited = [0.0f32; 3];
    for i in 0..3 {
        if !new_bias[i].is_finite() || !prev_bias[i].is_finite() {
            return Err(GyroBiasError::NonFinite);
        }
        let delta = new_bias[i] - prev_bias[i];
        if delta > max_delta {
            limited[i] = prev_bias[i] + max_delta;
        } else if delta < -max_delta {
            limited[i] = prev_bias[i] - max_delta;
        } else {
            limited[i] = new_bias[i];
        }
    }
    Ok(limited)
}

/// Perform one integral tracking step using rate/attitude error feedback.
pub fn track_bias_from_error(
    prev_bias: [f32; 3],
    error: [f32; 3],
    ki: f32,
    dt_s: f32,
    max_bias: f32,
) -> Result<[f32; 3], GyroBiasError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(GyroBiasError::InvalidSampleInterval);
    }
    if !ki.is_finite() || ki < 0.0 {
        return Err(GyroBiasError::InvalidGain);
    }
    if !max_bias.is_finite() || max_bias <= 0.0 {
        return Err(GyroBiasError::InvalidLimit);
    }
    for i in 0..3 {
        if !prev_bias[i].is_finite() || !error[i].is_finite() {
            return Err(GyroBiasError::NonFinite);
        }
    }
    let step = ki * dt_s;
    let raw_next = [
        prev_bias[0] - step * error[0],
        prev_bias[1] - step * error[1],
        prev_bias[2] - step * error[2],
    ];
    clamp_bias_vector(raw_next, max_bias)
}

/// Perform one complementary low-pass bias tracking step when stationary.
pub fn track_bias_stationary(
    prev_bias: [f32; 3],
    gyro_raw: [f32; 3],
    alpha: f32,
    max_bias: f32,
) -> Result<[f32; 3], GyroBiasError> {
    if !alpha.is_finite() || alpha <= 0.0 || alpha > 1.0 {
        return Err(GyroBiasError::InvalidGain);
    }
    if !max_bias.is_finite() || max_bias <= 0.0 {
        return Err(GyroBiasError::InvalidLimit);
    }
    for i in 0..3 {
        if !prev_bias[i].is_finite() || !gyro_raw[i].is_finite() {
            return Err(GyroBiasError::NonFinite);
        }
    }
    let updated = [
        prev_bias[0] + alpha * (gyro_raw[0] - prev_bias[0]),
        prev_bias[1] + alpha * (gyro_raw[1] - prev_bias[1]),
        prev_bias[2] + alpha * (gyro_raw[2] - prev_bias[2]),
    ];
    clamp_bias_vector(updated, max_bias)
}

/// Complementary filter gyroscope bias drift estimator and tracker.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroBiasTracker {
    config: GyroBiasConfig,
    bias_rad_s: [f32; 3],
    stationary_count: u32,
}

impl GyroBiasTracker {
    /// Construct tracker with specified configuration.
    pub fn new(config: GyroBiasConfig) -> Result<Self, GyroBiasError> {
        config.validate()?;
        Ok(Self {
            config,
            bias_rad_s: [0.0; 3],
            stationary_count: 0,
        })
    }
    /// Access tracker configuration.
    pub const fn config(&self) -> &GyroBiasConfig {
        &self.config
    }

    /// Get current estimated bias vector in rad/s.
    pub const fn bias(&self) -> [f32; 3] {
        self.bias_rad_s
    }

    /// Number of consecutive stationary updates processed.
    pub const fn stationary_count(&self) -> u32 {
        self.stationary_count
    }

    /// Set bias vector directly, bounded by configured maximum.
    pub fn set_bias(&mut self, bias: [f32; 3]) -> Result<(), GyroBiasError> {
        self.bias_rad_s = clamp_bias_vector(bias, self.config.max_bias_rad_s)?;
        Ok(())
    }

    /// Reset estimated bias and stationary counter to zero.
    pub fn reset(&mut self) {
        self.bias_rad_s = [0.0; 3];
        self.stationary_count = 0;
    }
    /// Correct a raw gyroscope sample by subtracting current bias.
    pub fn correct(&self, gyro_raw: [f32; 3]) -> Result<[f32; 3], GyroBiasError> {
        correct_gyro_sample(gyro_raw, self.bias_rad_s)
    }

    /// Determine whether a raw measurement indicates stationary conditions.
    pub fn is_stationary(&self, gyro_raw: [f32; 3]) -> bool {
        let corrected = match self.correct(gyro_raw) {
            Ok(c) => c,
            Err(_) => return false,
        };
        let mag_sq = corrected[0] * corrected[0]
            + corrected[1] * corrected[1]
            + corrected[2] * corrected[2];
        let thresh = self.config.stationary_threshold_rad_s;
        mag_sq <= thresh * thresh
    }
    /// Update bias estimate from rate/attitude error vector with slew limiting.
    pub fn update_from_error(
        &mut self,
        rate_error: [f32; 3],
        dt_s: f32,
    ) -> Result<[f32; 3], GyroBiasError> {
        let candidate = track_bias_from_error(
            self.bias_rad_s,
            rate_error,
            self.config.ki,
            dt_s,
            self.config.max_bias_rad_s,
        )?;
        let limited = apply_bias_slew_limit(
            candidate,
            self.bias_rad_s,
            self.config.max_drift_rate_rad_s2,
            dt_s,
        )?;
        self.bias_rad_s = limited;
        Ok(self.bias_rad_s)
    }
    /// Update bias estimate when stationary using complementary low-pass filter.
    pub fn update_stationary(
        &mut self,
        gyro_raw: [f32; 3],
        dt_s: f32,
    ) -> Result<bool, GyroBiasError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(GyroBiasError::InvalidSampleInterval);
        }
        if !self.is_stationary(gyro_raw) {
            self.stationary_count = 0;
            return Ok(false);
        }
        let candidate = track_bias_stationary(
            self.bias_rad_s,
            gyro_raw,
            self.config.stationary_learning_rate,
            self.config.max_bias_rad_s,
        )?;
        let limited = apply_bias_slew_limit(
            candidate,
            self.bias_rad_s,
            self.config.max_drift_rate_rad_s2,
            dt_s,
        )?;
        self.bias_rad_s = limited;
        self.stationary_count = self.stationary_count.saturating_add(1);
        Ok(true)
    }
    /// Update bias estimate by comparing raw gyro against reference rates.
    pub fn update_from_reference(
        &mut self,
        gyro_raw: [f32; 3],
        gyro_ref: [f32; 3],
        dt_s: f32,
    ) -> Result<[f32; 3], GyroBiasError> {
        for i in 0..3 {
            if !gyro_raw[i].is_finite() || !gyro_ref[i].is_finite() {
                return Err(GyroBiasError::NonFinite);
            }
        }
        let err = [
            gyro_ref[0] - (gyro_raw[0] - self.bias_rad_s[0]),
            gyro_ref[1] - (gyro_raw[1] - self.bias_rad_s[1]),
            gyro_ref[2] - (gyro_raw[2] - self.bias_rad_s[2]),
        ];
        self.update_from_error(err, dt_s)
    }
}
