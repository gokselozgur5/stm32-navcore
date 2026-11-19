//! 2-axis pitch/roll complementary filter in no-std.

/// Error returned when complementary filter parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComplementaryError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Alpha weight must be in (0, 1).
    InvalidAlpha,
}

/// 2-axis complementary filter for attitude estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComplementaryFilter {
    alpha: f32,
    dt: f32,
    roll: f32,
    pitch: f32,
}

impl ComplementaryFilter {
    /// Creates a new complementary filter with given alpha and dt.
    pub fn new(alpha: f32, dt: f32) -> Result<Self, ComplementaryError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(ComplementaryError::InvalidSampleInterval);
        }
        if !alpha.is_finite() || alpha <= 0.0 || alpha >= 1.0 {
            return Err(ComplementaryError::InvalidAlpha);
        }
        Ok(Self {
            alpha,
            dt,
            roll: 0.0,
            pitch: 0.0,
        })
    }

    /// Returns current roll angle estimate in radians.
    pub fn roll(&self) -> f32 {
        self.roll
    }

    /// Returns current pitch angle estimate in radians.
    pub fn pitch(&self) -> f32 {
        self.pitch
    }

    /// Resets filter state to zero.
    pub fn reset(&mut self) {
        self.roll = 0.0;
        self.pitch = 0.0;
    }

    /// Updates orientation using gyro rates (rad/s) and accelerometer angles (rad).
    pub fn update(
        &mut self,
        gyro_roll_rate: f32,
        gyro_pitch_rate: f32,
        accel_roll: f32,
        accel_pitch: f32,
    ) {
        let pred_roll = self.roll + gyro_roll_rate * self.dt;
        let pred_pitch = self.pitch + gyro_pitch_rate * self.dt;
        let roll_diff = wrap_pi(accel_roll - pred_roll);
        self.roll = wrap_pi(pred_roll + (1.0 - self.alpha) * roll_diff);
        self.pitch = self.alpha * pred_pitch + (1.0 - self.alpha) * accel_pitch;
    }
}

fn wrap_pi(rad: f32) -> f32 {
    let two_pi = 2.0 * core::f32::consts::PI;
    let mut wrapped = (rad + core::f32::consts::PI) % two_pi;
    if wrapped <= 0.0 {
        wrapped += two_pi;
    }
    wrapped - core::f32::consts::PI
}
