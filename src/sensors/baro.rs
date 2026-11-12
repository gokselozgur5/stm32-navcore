//! Barometer altitude conversion, compensation and filtering.

use crate::math::{pressure_hpa_to_altitude_m, MathError};

/// Sea-level standard pressure in hPa.
pub const ISA_SEA_LEVEL_HPA: f32 = 1013.25;

/// Altitude measurement the EKF can consume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroMeasurement {
    /// Altitude above the reference pressure level in meters.
    pub altitude_m: f32,
    /// Measurement variance in square meters.
    pub variance_m2: f32,
}

impl BaroMeasurement {
    /// Builds a measurement from pressure using the ISA model.
    pub fn from_pressure(p_hpa: f32, p0_hpa: f32, variance_m2: f32) -> Result<Self, MathError> {
        let altitude_m = pressure_hpa_to_altitude_m(p_hpa, p0_hpa)?;
        Ok(Self {
            altitude_m,
            variance_m2,
        })
    }
}

/// ISA sea-level temperature in kelvin.
pub const ISA_SEA_LEVEL_K: f32 = 288.15;

/// Error from temperature-compensated altitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BaroError {
    Pressure(MathError),
    Temperature,
}
