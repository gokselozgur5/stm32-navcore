//! Circular trajectory turn rate and bank angle aerodynamic calculator.

/// Errors in turn rate and aerodynamic calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TurnRateError {
    /// Input value is NaN or infinite.
    NonFinite,
    /// Airspeed is non-positive or negative.
    InvalidAirspeed,
    /// Turn radius is non-positive.
    InvalidRadius,
    /// Gravity acceleration is non-positive.
    InvalidGravity,
    /// Bank angle exceeds safe aerodynamic limits.
    BankAngleOutOfRange,
    /// Turn rate is zero when calculating radius.
    ZeroTurnRate,
    /// Stall speed is non-positive.
    InvalidStallSpeed,
}

/// Standard sea-level gravity acceleration in m/s^2.
pub const STANDARD_GRAVITY_MPS2: f32 = 9.80665;

/// Standard rate turn for civil aviation in rad/s (3 deg/s).
pub const STANDARD_RATE_TURN_RAD_S: f32 = 0.05235987756;

/// Standard rate turn in degrees per second (3 deg/s).
pub const STANDARD_RATE_TURN_DEG_S: f32 = 3.0;

/// Maximum allowable bank angle in radians (85 degrees).
pub const MAX_BANK_ANGLE_RAD: f32 = 1.4835298642;

/// Degrees to radians factor.
pub const DEG_TO_RAD: f32 = 0.01745329252;

/// Radians to degrees factor.
pub const RAD_TO_DEG: f32 = 57.295779513;

fn validate_finite(val: f32) -> Result<(), TurnRateError> {
    if !val.is_finite() {
        return Err(TurnRateError::NonFinite);
    }
    Ok(())
}

fn validate_speed_gravity(speed: f32, gravity: f32) -> Result<(), TurnRateError> {
    validate_finite(speed)?;
    validate_finite(gravity)?;
    if speed <= 0.0 {
        return Err(TurnRateError::InvalidAirspeed);
    }
    if gravity <= 0.0 {
        return Err(TurnRateError::InvalidGravity);
    }
    Ok(())
}
