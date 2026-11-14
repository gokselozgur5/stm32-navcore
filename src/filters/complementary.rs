//! Two-axis pitch/roll complementary filter (no-std).

/// Error for invalid complementary filter parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComplementaryError {
    /// Time constant tau must be positive and finite.
    InvalidTimeConstant,
    /// Sample interval dt must be positive and finite.
    InvalidSampleInterval,
}

/// Two-axis pitch and roll complementary filter.
#[derive(Debug, Clone, Copy)]
pub struct ComplementaryFilter {
    alpha: f32,
    pitch: f32,
    roll: f32,
    initialized: bool,
}

impl ComplementaryFilter {
    /// Create a new complementary filter from time constant tau and interval dt.
    pub fn new(tau_sec: f32, dt: f32) -> Result<Self, ComplementaryError> {
        if !tau_sec.is_finite() || tau_sec <= 0.0 {
            return Err(ComplementaryError::InvalidTimeConstant);
        }
        if !dt.is_finite() || dt <= 0.0 {
            return Err(ComplementaryError::InvalidSampleInterval);
        }
        let alpha = tau_sec / (tau_sec + dt);
        Ok(Self {
            alpha,
            pitch: 0.0,
            roll: 0.0,
            initialized: false,
        })
    }
}

impl ComplementaryFilter {
    /// Return the filter coefficient alpha.
    pub const fn alpha(&self) -> f32 {
        self.alpha
    }

    /// Return current estimated pitch angle in radians.
    pub const fn pitch(&self) -> f32 {
        self.pitch
    }

    /// Return current estimated roll angle in radians.
    pub const fn roll(&self) -> f32 {
        self.roll
    }

    /// Reset internal state to zero.
    pub fn reset(&mut self) {
        self.pitch = 0.0;
        self.roll = 0.0;
        self.initialized = false;
    }
}

impl ComplementaryFilter {
    /// Update attitude estimate with gyro rate and accel measurement.
    pub fn update(&mut self, pitch_rate: f32, roll_rate: f32, accel_pitch: f32, accel_roll: f32, dt: f32) -> (f32, f32) {
        if !self.initialized {
            self.pitch = accel_pitch;
            self.roll = accel_roll;
            self.initialized = true;
        } else {
            let gyro_pitch = self.pitch + pitch_rate * dt;
            let gyro_roll = self.roll + roll_rate * dt;
            self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch;
            self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll;
        }
        (self.pitch, self.roll)
    }
}
