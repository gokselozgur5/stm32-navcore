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
