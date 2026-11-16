//! True airspeed (TAS) estimation from pitot differential pressure and indicated airspeed (IAS).

/// Errors in pitot and airspeed calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PitotError {
    /// Input value is NaN or infinite.
    NonFinite,
    /// Static pressure is non-positive or unphysical.
    InvalidPressure,
    /// Absolute temperature is non-positive or below zero Kelvin.
    InvalidTemperature,
    /// Differential pressure is negative.
    NegativeDifferentialPressure,
    /// Indicated airspeed is negative.
    NegativeAirspeed,
    /// Calculated density is non-positive or non-finite.
    InvalidDensity,
}

/// Standard sea-level atmospheric pressure in Pascals (1013.25 hPa).
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in Kelvin (15.0 deg C).
pub const STANDARD_SEA_LEVEL_TEMP_K: f32 = 288.15;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_DRY_AIR: f32 = 287.05287;

/// Standard atmospheric air density at sea level in kg/m^3.
pub const STANDARD_SEA_LEVEL_DENSITY_KG_M3: f32 = 1.2250;
