//! Hypsometric barometric altitude estimator.

/// Errors that can occur during barometric altitude estimation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaroAltError {
    NonFinite,
    InvalidPressure,
    InvalidReferencePressure,
    InvalidTemperature,
    InvalidLapseRate,
}

/// Standard sea-level atmospheric pressure in Pascals (1013.25 hPa).
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in Kelvin (15.0 deg C).
pub const STANDARD_SEA_LEVEL_TEMP_K: f32 = 288.15;

/// Standard troposphere temperature lapse rate in K/m.
pub const STANDARD_LAPSE_RATE_K_PER_M: f32 = 0.0065;

/// Standard acceleration due to gravity in m/s^2.
pub const STANDARD_GRAVITY_M_S2: f32 = 9.80665;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_DRY_AIR: f32 = 287.05287;

/// Hypsometric exponent for standard atmosphere: (R * L) / g0.
pub const HYPSOMETRIC_EXPONENT: f32 =
    (GAS_CONSTANT_DRY_AIR * STANDARD_LAPSE_RATE_K_PER_M) / STANDARD_GRAVITY_M_S2;
