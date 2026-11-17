//! Barometric altitude hypsometric equation and temperature lapse compensator.

/// Errors occurring in hypsometric and lapse calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HypsometricError {
    /// Non-finite numeric input (NaN or Inf).
    NonFinite,
    /// Non-positive atmospheric pressure value.
    InvalidPressure,
    /// Absolute temperature below zero Kelvin.
    InvalidTemperature,
    /// Negative or excessive lapse rate.
    InvalidLapseRate,
    /// Division by zero or indeterminate ratio.
    ZeroDivision,
}

/// Standard sea-level pressure in Pascals (1013.25 hPa).
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in Kelvin (15.0 deg C).
pub const STANDARD_SEA_LEVEL_TEMP_K: f32 = 288.15;

/// Standard tropospheric temperature lapse rate in K/m (0.0065 K/m).
pub const STANDARD_LAPSE_RATE_K_PER_M: f32 = 0.0065;

/// Dry adiabatic lapse rate in K/m (~0.00976 K/m).
pub const DRY_ADIABATIC_LAPSE_RATE_K_PER_M: f32 = 0.00976;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_DRY_AIR: f32 = 287.05287;

/// Standard gravitational acceleration in m/s^2.
pub const STANDARD_GRAVITY_M_S2: f32 = 9.80665;

/// Validate pressure and temperature state parameters.
fn validate_state(p: f32, p_ref: f32, temp_k: f32) -> Result<(), HypsometricError> {
    if !p.is_finite() || !p_ref.is_finite() || !temp_k.is_finite() {
        return Err(HypsometricError::NonFinite);
    }
    if p <= 0.0 || p_ref <= 0.0 {
        return Err(HypsometricError::InvalidPressure);
    }
    if temp_k <= 0.0 {
        return Err(HypsometricError::InvalidTemperature);
    }
    Ok(())
}
