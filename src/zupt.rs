//! Zero-Velocity Update (ZUPT) stance detector for pedestrian and vehicular navigation.

/// Error conditions in ZUPT detection and configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZuptError {
    NonFinite,
    InvalidWindowSize,
    InvalidThreshold,
}

/// Stance phase state in zero-velocity detection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StancePhase {
    Moving,
    StanceCandidate,
    StanceConfirmed,
}

/// Configuration parameters for ZUPT stance detection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZuptConfig {
    /// Accelerometer variance threshold in (m/s^2)^2.
    pub accel_var_thresh: f32,
    /// Gyroscope angular rate norm threshold in rad/s.
    pub gyro_norm_thresh: f32,
    /// Accelerometer magnitude deviation threshold from gravity in m/s^2.
    pub accel_mag_thresh: f32,
    /// Minimum consecutive samples required to confirm stance.
    pub min_stance_samples: usize,
    /// Nominal gravity acceleration magnitude in m/s^2.
    pub gravity_magnitude: f32,
}

impl Default for ZuptConfig {
    fn default() -> Self {
        Self {
            accel_var_thresh: 0.05,
            gyro_norm_thresh: 0.05,
            accel_mag_thresh: 0.35,
            min_stance_samples: 5,
            gravity_magnitude: 9.80665,
        }
    }
}

impl ZuptConfig {
    /// Validates configuration thresholds.
    pub fn validate(&self) -> Result<(), ZuptError> {
        let finite = self.accel_var_thresh.is_finite()
            && self.gyro_norm_thresh.is_finite()
            && self.accel_mag_thresh.is_finite()
            && self.gravity_magnitude.is_finite();
        if !finite {
            return Err(ZuptError::NonFinite);
        }
        let positive = self.accel_var_thresh > 0.0
            && self.gyro_norm_thresh > 0.0
            && self.accel_mag_thresh > 0.0
            && self.gravity_magnitude > 0.0
            && self.min_stance_samples > 0;
        if !positive {
            return Err(ZuptError::InvalidThreshold);
        }
        Ok(())
    }
}

/// Detailed metrics computed for each zero-velocity detection step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZuptMetrics {
    /// Sample variance of accelerometer norm within window.
    pub accel_variance: f32,
    /// Maximum gyroscope norm within window.
    pub max_gyro_norm: f32,
    /// Absolute deviation of mean acceleration from nominal gravity.
    pub accel_mag_diff: f32,
    /// Combined test statistic ratio relative to thresholds.
    pub test_statistic: f32,
    /// Current detected stance phase.
    pub phase: StancePhase,
}

#[inline]
pub(crate) fn vec3_norm(v: [f32; 3]) -> f32 {
    libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

#[inline]
pub(crate) fn is_finite_vec3(v: [f32; 3]) -> bool {
    v[0].is_finite() && v[1].is_finite() && v[2].is_finite()
}
