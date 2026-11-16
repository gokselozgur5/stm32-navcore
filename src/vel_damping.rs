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
