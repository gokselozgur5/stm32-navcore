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

        // c01: 16-bit signed: byte8 + byte9
        let c01 = (((bytes[8] as u16) << 8) | (bytes[9] as u16)) as i16 as f32;

        // c11: 16-bit signed: byte10 + byte11
        let c11 = (((bytes[10] as u16) << 8) | (bytes[11] as u16)) as i16 as f32;

        // c20: 16-bit signed: byte12 + byte13
        let c20 = (((bytes[12] as u16) << 8) | (bytes[13] as u16)) as i16 as f32;

        // c21: 16-bit signed: byte14 + byte15
        let c21 = (((bytes[14] as u16) << 8) | (bytes[15] as u16)) as i16 as f32;

        // c30: 16-bit signed: byte16 + byte17
        let c30 = (((bytes[16] as u16) << 8) | (bytes[17] as u16)) as i16 as f32;

        Ok(Self {
            c0,
            c1,
            c00,
            c10,
            c01,
            c11,
            c20,
            c21,
            c30,
        })
    }
}

/// Sign-extends a 12-bit signed integer.
fn sign_extend_12(val: u16) -> i32 {
    let mut v = (val & 0x0FFF) as i32;
    if (v & 0x0800) != 0 {
        v -= 0x1000;
    }
    v
}

/// Sign-extends a 20-bit signed integer.
fn sign_extend_20(val: u32) -> i32 {
    let mut v = (val & 0x000F_FFFF) as i32;
    if (v & 0x0008_0000) != 0 {
        v -= 0x0010_0000;
    }
    v
}

/// Sign-extends a 24-bit signed integer.
fn sign_extend_24(val: u32) -> i32 {
    let mut v = (val & 0x00FF_FFFF) as i32;
    if (v & 0x0080_0000) != 0 {
        v -= 0x0100_0000;
    }
    v
}

/// Zero-alloc driver and DSP engine for Infineon DPS310.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dps310Driver {
    pub calib: Dps310Calib,
    pub p_scale: f32,
    pub t_scale: f32,
}

impl Default for Dps310Driver {
    fn default() -> Self {
        Self::new(Dps310Calib::default())
    }
}

impl Dps310Driver {
    /// Creates a new DPS310 driver instance with default standard scaling (kP=524288, kT=524288).
    pub const fn new(calib: Dps310Calib) -> Self {
        Self {
            calib,
            p_scale: 524288.0,
            t_scale: 524288.0,
        }
    }

    /// Sets the temperature oversampling scale factor (e.g., 524288 for 1x, 1572864 for 4x, etc.).
    pub fn set_temperature_scale(&mut self, scale: f32) {
        if scale > 0.0 {
            self.t_scale = scale;
        }
    }

    /// Sets the pressure oversampling scale factor (e.g., 524288 for 1x, 3670016 for 8x, etc.).
    pub fn set_pressure_scale(&mut self, scale: f32) {
        if scale > 0.0 {
            self.p_scale = scale;
        }
