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
