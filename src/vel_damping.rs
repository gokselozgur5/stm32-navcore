//! Dead reckoning velocity damping and GPS velocity fusion.

/// Errors returned by velocity damping and GPS fusion operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VelDampingError {
    /// Sample interval must be finite and strictly positive.
    InvalidSampleInterval,
    /// Damping time constant must be finite and strictly positive.
    InvalidTimeConstant,
    /// GPS fusion gain must be finite and within [0.0, 1.0].
    InvalidGain,
    /// Gate threshold must be finite and strictly positive.
    InvalidGateThreshold,
    /// Non-finite vector or scalar input.
    NonFinite,
    /// Innovation exceeded the statistical gating threshold.
    InnovationGateExceeded,
}

/// Configuration parameters for velocity damping and GPS fusion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelDampingConfig {
    /// Time constant tau in seconds for exponential damping.
    pub tau_s: f32,
    /// Complementary filter gain for GPS velocity fusion in [0.0, 1.0].
    pub gps_gain: f32,
    /// Maximum allowable innovation norm in m/s.
    pub max_innovation_mps: f32,
    /// Duration in seconds without GPS before marking fix invalid.
    pub gps_timeout_s: f32,
}

impl VelDampingConfig {
    /// Validate configuration invariants.
    pub fn validate(&self) -> Result<(), VelDampingError> {
        if !self.tau_s.is_finite() || self.tau_s <= 0.0 {
            return Err(VelDampingError::InvalidTimeConstant);
        }
        if !self.gps_gain.is_finite() || self.gps_gain < 0.0 || self.gps_gain > 1.0 {
            return Err(VelDampingError::InvalidGain);
        }
        if !self.max_innovation_mps.is_finite() || self.max_innovation_mps <= 0.0 {
            return Err(VelDampingError::InvalidGateThreshold);
        }
        if !self.gps_timeout_s.is_finite() || self.gps_timeout_s <= 0.0 {
            return Err(VelDampingError::InvalidSampleInterval);
        }
        Ok(())
    }
}

impl Default for VelDampingConfig {
    fn default() -> Self {
        Self {
            tau_s: 60.0,
            gps_gain: 0.1,
            max_innovation_mps: 5.0,
            gps_timeout_s: 2.0,
        }
    }
}
