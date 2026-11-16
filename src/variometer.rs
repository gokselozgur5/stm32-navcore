//! Barometric vertical speed and variometer estimator.

/// Errors returned by variometer operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VariometerError {
    /// Non-finite input (NaN or infinite).
    NonFinite,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
    /// Pressure must be positive and finite.
    InvalidPressure,
    /// Time constant must be strictly positive and finite.
    InvalidTimeConstant,
    /// Threshold values must be finite and consistent (climb >= sink).
    InvalidThreshold,
}

/// Factor to convert meters per second to feet per minute (60 / 0.3048).
pub const MPS_TO_FPM: f32 = 196.85039;

/// Factor to convert meters per second to knots (1 / 0.514444).
pub const MPS_TO_KNOTS: f32 = 1.9438445;
