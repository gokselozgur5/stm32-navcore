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

impl Dps310Calib {
    /// Decodes factory calibration coefficients from 18 raw bytes (registers 0x10..=0x21).
    pub fn from_raw_bytes(bytes: &[u8]) -> Result<Self, SensorError> {
        if bytes.len() < 18 {
            return Err(SensorError::BufferTooShort);
        }

        // c0: 12-bit signed: byte0[7:0] + byte1[7:4]
        let c0_raw = ((bytes[0] as u16) << 4) | ((bytes[1] as u16) >> 4);
        let c0 = sign_extend_12(c0_raw) as f32;

        // c1: 12-bit signed: byte1[3:0] + byte2[7:0]
        let c1_raw = ((bytes[1] as u16 & 0x0F) << 8) | (bytes[2] as u16);
        let c1 = sign_extend_12(c1_raw) as f32;

        // c00: 20-bit signed: byte3[7:0] + byte4[7:0] + byte5[7:4]
        let c00_raw = ((bytes[3] as u32) << 12) | ((bytes[4] as u32) << 4) | ((bytes[5] as u32) >> 4);
        let c00 = sign_extend_20(c00_raw) as f32;

        // c10: 20-bit signed: byte5[3:0] + byte6[7:0] + byte7[7:0]
        let c10_raw = (((bytes[5] as u32) & 0x0F) << 16) | ((bytes[6] as u32) << 8) | (bytes[7] as u32);
        let c10 = sign_extend_20(c10_raw) as f32;

