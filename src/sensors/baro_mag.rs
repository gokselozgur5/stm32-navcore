//! High-Precision DPS310 Barometer and IST8310 Magnetometer Codecs.
//!
//! Provides zero-allocation, deterministic parsing and compensation for:
//! - Infineon DPS310 (24-bit pressure & temperature with factory polynomial compensation).
//! - iSentek IST8310 (16-bit 3-axis magnetometer with cross-axis sensitivity matrix correction).

use crate::math::{pressure_hpa_to_altitude_m, tilt_compensated_yaw, Mat, MathError};

/// Sensor errors during decoding, parsing, or compensation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorError {
    /// Calibration coefficient or register buffer length was insufficient.
    BufferTooShort,
    /// Parsed reading or compensated value was non-finite.
    NonFinite,
    /// Barometric pressure was zero, negative, or invalid.
    InvalidPressure,
    /// Magnetometer magnetic vector norm was invalid or zero.
    InvalidMagNorm,
    /// Mathematical error during heading or altitude calculation.
    Math(MathError),
}

impl From<MathError> for SensorError {
    fn from(err: MathError) -> Self {
        SensorError::Math(err)
    }
}

/// DPS310 factory calibration coefficients.
///
/// Registers 0x10 to 0x21 contain 18 bytes of calibration data:
/// c0 (12-bit), c1 (12-bit), c00 (20-bit), c10 (20-bit),
/// c01 (16-bit), c11 (16-bit), c20 (16-bit), c21 (16-bit), c30 (16-bit).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Dps310Calib {
    pub c0: f32,
    pub c1: f32,
    pub c00: f32,
    pub c10: f32,
    pub c01: f32,
    pub c11: f32,
    pub c20: f32,
    pub c21: f32,
    pub c30: f32,
}

