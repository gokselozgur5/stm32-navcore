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
