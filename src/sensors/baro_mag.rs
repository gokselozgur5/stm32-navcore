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
    }

    /// Decodes raw 24-bit 2's complement temperature from 3 register bytes (0x03, 0x04, 0x05).
    pub fn parse_raw_temperature(bytes: &[u8]) -> Result<i32, SensorError> {
        if bytes.len() < 3 {
            return Err(SensorError::BufferTooShort);
        }
        let raw = ((bytes[0] as u32) << 16) | ((bytes[1] as u32) << 8) | (bytes[2] as u32);
        Ok(sign_extend_24(raw))
    }

    /// Decodes raw 24-bit 2's complement pressure from 3 register bytes (0x00, 0x01, 0x02).
    pub fn parse_raw_pressure(bytes: &[u8]) -> Result<i32, SensorError> {
        if bytes.len() < 3 {
            return Err(SensorError::BufferTooShort);
        }
        let raw = ((bytes[0] as u32) << 16) | ((bytes[1] as u32) << 8) | (bytes[2] as u32);
        Ok(sign_extend_24(raw))
    }

    /// Calculates scaled temperature T_raw / kT.
    pub fn scale_temperature(&self, raw_t: i32) -> f32 {
        (raw_t as f32) / self.t_scale
    }

    /// Calculates scaled pressure P_raw / kP.
    pub fn scale_pressure(&self, raw_p: i32) -> f32 {
        (raw_p as f32) / self.p_scale
    }

    /// Compensates temperature in degrees Celsius (°C).
    ///
    /// Formula: T_comp = c0 * 0.5 + c1 * T_raw_sc.
    pub fn compensate_temperature(&self, raw_t: i32) -> Result<f32, SensorError> {
        let t_raw_sc = self.scale_temperature(raw_t);
        let t_comp = self.calib.c0 * 0.5 + self.calib.c1 * t_raw_sc;
        if !t_comp.is_finite() {
            return Err(SensorError::NonFinite);
        }
        Ok(t_comp)
    }

    /// Compensates pressure in Pascals (Pa).
    ///
    /// Polynomial:
    /// P_comp = c00 + P_sc * (c10 + P_sc * (c20 + P_sc * c30)) +
    ///          T_sc * c01 + T_sc * P_sc * (c11 + P_sc * c21).
    pub fn compensate_pressure(&self, raw_p: i32, raw_t: i32) -> Result<f32, SensorError> {
        let p_sc = self.scale_pressure(raw_p);
        let t_sc = self.scale_temperature(raw_t);

        let term_p = p_sc * (self.calib.c10 + p_sc * (self.calib.c20 + p_sc * self.calib.c30));
        let term_t = t_sc * self.calib.c01;
        let term_tp = t_sc * p_sc * (self.calib.c11 + p_sc * self.calib.c21);

        let p_comp = self.calib.c00 + term_p + term_t + term_tp;
        if !p_comp.is_finite() || p_comp <= 0.0 {
            return Err(SensorError::InvalidPressure);
        }
        Ok(p_comp)
    }

    /// Compensates pressure in hectopascals (hPa).
    pub fn compensate_pressure_hpa(&self, raw_p: i32, raw_t: i32) -> Result<f32, SensorError> {
        let p_pa = self.compensate_pressure(raw_p, raw_t)?;
        Ok(p_pa * 0.01)
    }

    /// Computes pressure altitude in meters relative to standard sea level pressure or given p0_hpa.
    pub fn pressure_altitude_m(
        &self,
        raw_p: i32,
        raw_t: i32,
        p0_hpa: f32,
    ) -> Result<f32, SensorError> {
        let p_hpa = self.compensate_pressure_hpa(raw_p, raw_t)?;
        let alt = pressure_hpa_to_altitude_m(p_hpa, p0_hpa)?;
        Ok(alt)
    }
}

/// Zero-alloc driver and DSP engine for iSentek IST8310 3-axis magnetometer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ist8310Driver {
    /// Cross-axis sensitivity matrix (3x3 soft-iron and non-orthogonality correction).
    pub sensitivity_matrix: Mat<3, 3>,
    /// Hard-iron bias offset vector [Bx, By, Bz] in microteslas (uT).
    pub bias: [f32; 3],
    /// Sensitivity scale factor (uT per LSB). IST8310 default is 0.3 uT/LSB (3300 LSB/Gauss).
    pub scale_ut_per_lsb: f32,
}

impl Default for Ist8310Driver {
    fn default() -> Self {
        Self {
            sensitivity_matrix: Mat::identity(),
            bias: [0.0; 3],
            scale_ut_per_lsb: 0.3,
        }
    }
}

impl Ist8310Driver {
    /// Creates a new IST8310 driver with default 0.3 uT/LSB scale and identity cross-axis matrix.
    pub fn new() -> Self {
        Self {
            sensitivity_matrix: Mat::identity(),
            bias: [0.0; 3],
            scale_ut_per_lsb: 0.3,
        }
    }

    /// Sets cross-axis sensitivity correction matrix.
    pub fn set_cross_axis_matrix(&mut self, matrix: Mat<3, 3>) {
        self.sensitivity_matrix = matrix;
    }

    /// Sets hard-iron bias offset vector [Bx, By, Bz] in microteslas (uT).
    pub fn set_bias(&mut self, bias: [f32; 3]) {
        self.bias = bias;
    }

    /// Sets custom scale in uT per LSB.
    pub fn set_scale_ut_per_lsb(&mut self, scale: f32) {
        if scale > 0.0 {
            self.scale_ut_per_lsb = scale;
        }
    }

    /// Parses 16-bit little-endian 2's complement raw magnetic vector from 6 register bytes (0x03..=0x08).
    ///
    /// Register layout:
    /// - 0x03: X low byte, 0x04: X high byte
    /// - 0x05: Y low byte, 0x06: Y high byte
    /// - 0x07: Z low byte, 0x08: Z high byte
    pub fn parse_raw_mag(bytes: &[u8]) -> Result<[i16; 3], SensorError> {
        if bytes.len() < 6 {
            return Err(SensorError::BufferTooShort);
        }
        let x = i16::from_le_bytes([bytes[0], bytes[1]]);
        let y = i16::from_le_bytes([bytes[2], bytes[3]]);
        let z = i16::from_le_bytes([bytes[4], bytes[5]]);
        Ok([x, y, z])
    }

    /// Converts raw counts to calibrated magnetic field vector [uT] applying scale,
    /// hard-iron bias removal, and cross-axis sensitivity matrix correction.
    pub fn compensate_mag(&self, raw: [i16; 3]) -> Result<[f32; 3], SensorError> {
        let bx_unscaled = (raw[0] as f32) * self.scale_ut_per_lsb - self.bias[0];
        let by_unscaled = (raw[1] as f32) * self.scale_ut_per_lsb - self.bias[1];
        let bz_unscaled = (raw[2] as f32) * self.scale_ut_per_lsb - self.bias[2];

        // Apply cross-axis sensitivity correction: B_cal = S * B_unscaled
        let s = self.sensitivity_matrix.0;
        let bx = s[0][0] * bx_unscaled + s[0][1] * by_unscaled + s[0][2] * bz_unscaled;
        let by = s[1][0] * bx_unscaled + s[1][1] * by_unscaled + s[1][2] * bz_unscaled;
        let bz = s[2][0] * bx_unscaled + s[2][1] * by_unscaled + s[2][2] * bz_unscaled;

        if !bx.is_finite() || !by.is_finite() || !bz.is_finite() {
            return Err(SensorError::NonFinite);
        }

        let norm_sq = bx * bx + by * by + bz * bz;
        if norm_sq < 1e-6 {
            return Err(SensorError::InvalidMagNorm);
        }

        Ok([bx, by, bz])
    }

    /// Computes tilt-compensated heading (yaw in radians [-pi, pi]) from calibrated magnetic vector
    /// and roll/pitch attitude angles (radians).
    pub fn tilt_compensated_yaw(
        &self,
        raw: [i16; 3],
        roll: f32,
        pitch: f32,
    ) -> Result<f32, SensorError> {
        let b_scaled = [
            (raw[0] as f32) * self.scale_ut_per_lsb,
            (raw[1] as f32) * self.scale_ut_per_lsb,
            (raw[2] as f32) * self.scale_ut_per_lsb,
        ];
        let yaw = tilt_compensated_yaw(b_scaled, self.bias, self.sensitivity_matrix, roll, pitch)?;
        Ok(yaw)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dps310_sign_extension() {
        assert_eq!(sign_extend_12(0x0001), 1);
        assert_eq!(sign_extend_12(0x07FF), 2047);
        assert_eq!(sign_extend_12(0x0800), -2048);
        assert_eq!(sign_extend_12(0x0FFF), -1);

        assert_eq!(sign_extend_20(0x00001), 1);
        assert_eq!(sign_extend_20(0x7FFFF), 524287);
        assert_eq!(sign_extend_20(0x80000), -524288);
        assert_eq!(sign_extend_20(0xFFFFF), -1);

        assert_eq!(sign_extend_24(0x000001), 1);
        assert_eq!(sign_extend_24(0x7FFFFF), 8388607);
        assert_eq!(sign_extend_24(0x800000), -8388608);
        assert_eq!(sign_extend_24(0xFFFFFF), -1);
    }

    #[test]
    fn test_dps310_calib_decoding_and_buffer_bounds() {
        assert_eq!(
            Dps310Calib::from_raw_bytes(&[0u8; 17]),
            Err(SensorError::BufferTooShort)
        );

        // Dummy known bytes
        let mut raw = [0u8; 18];
        raw[0] = 0x12;
        raw[1] = 0x34;
        raw[2] = 0x56;
        let cal = Dps310Calib::from_raw_bytes(&raw).unwrap();
        assert_eq!(cal.c0, 0x123 as f32);
        assert_eq!(cal.c1, 0x456 as f32);
    }

    #[test]
    fn test_dps310_pressure_temperature_compensation() {
        // Known calibration parameters
        let calib = Dps310Calib {
            c0: 40.0,
            c1: 1000.0,
            c00: 101325.0,
            c10: 200.0,
            c01: -50.0,
            c11: 10.0,
            c20: 5.0,
            c21: 1.0,
            c30: 0.1,
        };
        let driver = Dps310Driver::new(calib);

        let t_raw = 524288; // T_sc = 1.0
        let p_raw = 524288; // P_sc = 1.0

        let temp_c = driver.compensate_temperature(t_raw).unwrap();
        assert!((temp_c - (40.0 * 0.5 + 1000.0 * 1.0)).abs() < 1e-4);

        let p_pa = driver.compensate_pressure(p_raw, t_raw).unwrap();
        assert!(p_pa > 100000.0 && p_pa < 105000.0);

        let alt_m = driver.pressure_altitude_m(p_raw, t_raw, 1013.25).unwrap();
