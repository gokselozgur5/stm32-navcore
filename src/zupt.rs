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

/// Fixed-window zero-velocity update detector for embedded Cortex-M platforms.
#[derive(Clone, Debug, PartialEq)]
pub struct ZuptDetector<const WINDOW: usize> {
    config: ZuptConfig,
    count: usize,
    head: usize,
    stance_streak: usize,
    accel_buf: [[f32; 3]; WINDOW],
    gyro_buf: [[f32; 3]; WINDOW],
    current_metrics: ZuptMetrics,
}

impl<const WINDOW: usize> ZuptDetector<WINDOW> {
    /// Constructs a new detector with given configuration.
    pub fn new(config: ZuptConfig) -> Result<Self, ZuptError> {
        if WINDOW < 2 {
            return Err(ZuptError::InvalidWindowSize);
        }
        config.validate()?;
        let initial_metrics = ZuptMetrics {
            accel_variance: 0.0,
            max_gyro_norm: 0.0,
            accel_mag_diff: 0.0,
            test_statistic: 0.0,
            phase: StancePhase::Moving,
        };
        Ok(Self {
            config,
            count: 0,
            head: 0,
            stance_streak: 0,
            accel_buf: [[0.0; 3]; WINDOW],
            gyro_buf: [[0.0; 3]; WINDOW],
            current_metrics: initial_metrics,
        })
    }

    /// Resets internal buffer and detector state.
    pub fn reset(&mut self) {
        self.count = 0;
        self.head = 0;
        self.stance_streak = 0;
        self.accel_buf = [[0.0; 3]; WINDOW];
        self.gyro_buf = [[0.0; 3]; WINDOW];
        self.current_metrics = ZuptMetrics {
            accel_variance: 0.0,
            max_gyro_norm: 0.0,
            accel_mag_diff: 0.0,
            test_statistic: 0.0,
            phase: StancePhase::Moving,
        };
    }

    fn compute_accel_stats(&self, valid_len: usize) -> (f32, f32) {
        let mut sum_mag = 0.0f32;
        let mut sum_mag_sq = 0.0f32;
        for i in 0..valid_len {
            let mag = vec3_norm(self.accel_buf[i]);
            sum_mag += mag;
            sum_mag_sq += mag * mag;
        }
        let inv_n = 1.0 / (valid_len as f32);
        let mean = sum_mag * inv_n;
        let variance = (sum_mag_sq * inv_n) - (mean * mean);
        let non_neg_variance = if variance < 0.0 { 0.0 } else { variance };
        let mag_diff = (mean - self.config.gravity_magnitude).abs();
        (non_neg_variance, mag_diff)
    }

    fn compute_gyro_stats(&self, valid_len: usize) -> f32 {
        let mut max_norm = 0.0f32;
        for i in 0..valid_len {
            let norm = vec3_norm(self.gyro_buf[i]);
            if norm > max_norm {
                max_norm = norm;
            }
        }
        max_norm
    }

    fn push_sample(&mut self, accel: [f32; 3], gyro: [f32; 3]) {
        self.accel_buf[self.head] = accel;
        self.gyro_buf[self.head] = gyro;
        self.head = (self.head + 1) % WINDOW;
        if self.count < WINDOW {
            self.count += 1;
        }
    }

    fn evaluate_stance_criteria(&self, var_a: f32, max_w: f32, diff_a: f32) -> (bool, f32) {
        let ratio_var = var_a / self.config.accel_var_thresh;
        let ratio_w = max_w / self.config.gyro_norm_thresh;
        let ratio_diff = diff_a / self.config.accel_mag_thresh;
        let stat = ratio_var + ratio_w + ratio_diff;
        let meets = var_a <= self.config.accel_var_thresh
            && max_w <= self.config.gyro_norm_thresh
            && diff_a <= self.config.accel_mag_thresh;
        (meets, stat)
    }

    fn update_stance_streak(&mut self, criteria_met: bool) -> StancePhase {
        if criteria_met {
            self.stance_streak += 1;
            if self.stance_streak >= self.config.min_stance_samples {
                StancePhase::StanceConfirmed
            } else {
                StancePhase::StanceCandidate
            }
        } else {
            self.stance_streak = 0;
            StancePhase::Moving
        }
    }

    /// Ingests IMU sample and returns whether stance is currently confirmed.
    pub fn update(&mut self, accel: [f32; 3], gyro: [f32; 3]) -> Result<bool, ZuptError> {
        if !is_finite_vec3(accel) || !is_finite_vec3(gyro) {
            return Err(ZuptError::NonFinite);
        }
        self.push_sample(accel, gyro);
        if self.count < WINDOW {
            self.current_metrics.phase = StancePhase::Moving;
            return Ok(false);
        }
        let (var_a, diff_a) = self.compute_accel_stats(WINDOW);
        let max_w = self.compute_gyro_stats(WINDOW);
        let (meets, stat) = self.evaluate_stance_criteria(var_a, max_w, diff_a);
        let phase = self.update_stance_streak(meets);
        self.current_metrics = ZuptMetrics {
            accel_variance: var_a,
            max_gyro_norm: max_w,
            accel_mag_diff: diff_a,
            test_statistic: stat,
            phase,
        };
        Ok(phase == StancePhase::StanceConfirmed)
    }

    /// Returns reference to most recent metrics.
    pub fn metrics(&self) -> &ZuptMetrics {
        &self.current_metrics
    }

    /// Returns current detected stance phase.
    pub fn phase(&self) -> StancePhase {
        self.current_metrics.phase
    }

    /// Returns true if stance is confirmed.
    pub fn is_stance(&self) -> bool {
        self.current_metrics.phase == StancePhase::StanceConfirmed
    }

    /// Returns reference to active detector configuration.
    pub fn config(&self) -> &ZuptConfig {
        &self.config
    }
}

/// Default ZUPT detector with standard 16-sample sliding window.
pub type DefaultZuptDetector = ZuptDetector<16>;

impl<const WINDOW: usize> Default for ZuptDetector<WINDOW> {
    fn default() -> Self {
        Self::new(ZuptConfig::default()).expect("default ZuptConfig must be valid")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_static_detector_detects_stance() {
        let mut detector = ZuptDetector::<8>::default();
        let accel = [0.0, 0.0, -9.80665];
        let gyro = [0.001, -0.001, 0.0005];
        let mut confirmed = false;
        for _ in 0..15 {
            confirmed = detector.update(accel, gyro).unwrap();
        }
        assert!(confirmed);
        assert_eq!(detector.phase(), StancePhase::StanceConfirmed);
        assert!(detector.metrics().accel_variance < 1e-4);
    }

    #[test]
    fn test_dynamic_motion_rejects_stance() {
        let mut detector = ZuptDetector::<8>::default();
        let accel = [1.5, 0.0, -9.8];
        let gyro = [0.2, -0.15, 0.1];
        let mut confirmed = false;
        for _ in 0..15 {
            confirmed = detector.update(accel, gyro).unwrap();
        }
        assert!(!confirmed);
        assert_eq!(detector.phase(), StancePhase::Moving);
        assert!(!detector.is_stance());
    }
}
