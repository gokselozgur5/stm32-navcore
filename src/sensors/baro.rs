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
