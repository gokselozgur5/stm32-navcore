//! Pitot-static indicated airspeed (IAS) and true airspeed (TAS) calculator.

/// Errors occurring in airspeed and atmospheric computations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirspeedError {
    /// Non-finite input parameter.
    NonFinite,
    /// Absolute or static pressure is non-positive.
    InvalidPressure,
    /// Absolute temperature is non-positive.
    InvalidTemperature,
    /// Differential pressure is negative.
    NegativeDifferentialPressure,
    /// Calculated or supplied density is non-positive.
    InvalidDensity,
    /// Airspeed is negative.
    NegativeAirspeed,
    /// Subsonic model exceeded (Mach >= 1.0).
    SupersonicFlow,
}

/// Standard sea-level pressure in Pa.
pub const STANDARD_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in K.
pub const STANDARD_TEMP_K: f32 = 288.15;

/// Standard sea-level density in kg/m^3.
pub const STANDARD_DENSITY_KG_M3: f32 = 1.2250;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_AIR: f32 = 287.05287;

/// Heat capacity ratio for air.
pub const HEAT_CAPACITY_RATIO: f32 = 1.4;

/// Standard speed of sound in m/s.
pub const STANDARD_SPEED_OF_SOUND_MPS: f32 = 340.294;

/// Tropospheric lapse rate in K/m.
