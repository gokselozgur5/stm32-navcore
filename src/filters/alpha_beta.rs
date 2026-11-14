//! Alpha-beta tracker for scalar position and velocity (NC-021).

/// Error returned when tracker parameters are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlphaBetaError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Alpha smoothing factor must be in (0, 1].
    InvalidAlpha,
    /// Beta velocity factor must be non-negative and satisfy stability bounds.
    InvalidBeta,
}

/// Alpha-beta tracker for scalar position and velocity tracking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlphaBetaFilter {
    alpha: f32,
    beta: f32,
    dt: f32,
    x: f32,
    v: f32,
    initialized: bool,
}

impl AlphaBetaFilter {
    /// Construct a new tracker with gains and sample interval.
    pub fn new(alpha: f32, beta: f32, dt: f32) -> Result<Self, AlphaBetaError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(AlphaBetaError::InvalidSampleInterval);
        }
        if !alpha.is_finite() || alpha <= 0.0 || alpha > 1.0 {
            return Err(AlphaBetaError::InvalidAlpha);
        }
        if !beta.is_finite() || beta <= 0.0 || beta > 2.0 * (2.0 - alpha) {
            return Err(AlphaBetaError::InvalidBeta);
        }
        Ok(Self {
            alpha,
            beta,
            dt,
            x: 0.0,
            v: 0.0,
            initialized: false,
        })
    }

    /// Return estimated position.
    pub const fn position(&self) -> f32 {
        self.x
    }

    /// Return estimated velocity.
    pub const fn velocity(&self) -> f32 {
        self.v
    }

    /// Return whether tracker has received at least one measurement.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Reset tracker state to uninitialized zero.
    pub fn reset(&mut self) {
        self.x = 0.0;
        self.v = 0.0;
        self.initialized = false;
    }

    /// Predict next state across elapsed time interval `dt`.
    pub fn predict(&mut self) {
        self.x += self.v * self.dt;
    }

    /// Update tracker with a new position measurement.
    pub fn update(&mut self, measurement: f32) -> (f32, f32) {
        if !self.initialized {
            self.x = measurement;
            self.v = 0.0;
            self.initialized = true;
        } else {
            self.predict();
            let residual = measurement - self.x;
            self.x += self.alpha * residual;
            self.v += (self.beta / self.dt) * residual;
        }
        (self.x, self.v)
    }
}




