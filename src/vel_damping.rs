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

/// Apply first-order exponential velocity damping step.
pub fn damp_velocity_step(
    vel: [f32; 3],
    tau_s: f32,
    dt_s: f32,
) -> Result<[f32; 3], VelDampingError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(VelDampingError::InvalidSampleInterval);
    }
    if !tau_s.is_finite() || tau_s <= 0.0 {
        return Err(VelDampingError::InvalidTimeConstant);
    }
    if !vel[0].is_finite() || !vel[1].is_finite() || !vel[2].is_finite() {
        return Err(VelDampingError::NonFinite);
    }
    let decay = libm::expf(-dt_s / tau_s);
    Ok([vel[0] * decay, vel[1] * decay, vel[2] * decay])
}

/// Compute innovation vector (GPS - DR) and Euclidean magnitude.
pub fn calculate_innovation(
    vel_dr: [f32; 3],
    vel_gps: [f32; 3],
) -> Result<([f32; 3], f32), VelDampingError> {
    for i in 0..3 {
        if !vel_dr[i].is_finite() || !vel_gps[i].is_finite() {
            return Err(VelDampingError::NonFinite);
        }
    }
    let innov = [
        vel_gps[0] - vel_dr[0],
        vel_gps[1] - vel_dr[1],
        vel_gps[2] - vel_dr[2],
    ];
    let norm = libm::sqrtf(
        innov[0] * innov[0] + innov[1] * innov[1] + innov[2] * innov[2],
    );
    Ok((innov, norm))
}

/// Fuse dead reckoning velocity with GPS velocity using proportional gain.
pub fn fuse_velocity_step(
    vel_dr: [f32; 3],
    vel_gps: [f32; 3],
    gain: f32,
    max_gate: f32,
) -> Result<[f32; 3], VelDampingError> {
    if !gain.is_finite() || gain < 0.0 || gain > 1.0 {
        return Err(VelDampingError::InvalidGain);
    }
    if !max_gate.is_finite() || max_gate <= 0.0 {
        return Err(VelDampingError::InvalidGateThreshold);
    }
    let (innov, norm) = calculate_innovation(vel_dr, vel_gps)?;
    if norm > max_gate {
        return Err(VelDampingError::InnovationGateExceeded);
    }
    Ok([
        vel_dr[0] + gain * innov[0],
        vel_dr[1] + gain * innov[1],
        vel_dr[2] + gain * innov[2],
    ])
}

/// Dead reckoning velocity filter with damping and GPS velocity fusion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelDampingFilter {
    config: VelDampingConfig,
    vel_ned: [f32; 3],
    time_since_gps_s: f32,
    gps_valid: bool,
}

impl VelDampingFilter {
    /// Construct a new filter with zero initial velocity.
    pub fn new(config: VelDampingConfig) -> Result<Self, VelDampingError> {
        Self::with_initial_velocity(config, [0.0, 0.0, 0.0])
    }

    /// Construct a new filter with specified initial velocity.
    pub fn with_initial_velocity(
        config: VelDampingConfig,
        initial_vel: [f32; 3],
    ) -> Result<Self, VelDampingError> {
        config.validate()?;
        for &v in &initial_vel {
            if !v.is_finite() {
                return Err(VelDampingError::NonFinite);
            }
        }
        Ok(Self {
            config,
            vel_ned: initial_vel,
            time_since_gps_s: 0.0,
            gps_valid: false,
        })
    }
}
