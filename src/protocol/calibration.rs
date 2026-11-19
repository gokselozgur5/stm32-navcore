use crate::protocol::codec::crc16_ccitt;
use crate::protocol::ProtocolError;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationBlock {
    pub version: u16,
    pub mag_hard_iron: [f32; 3],
    pub mag_soft_iron: [[f32; 3]; 3],
    pub gyro_bias_rps: [f32; 3],
    pub accel_scale: [f32; 3],
}

impl CalibrationBlock {
    pub const MAGIC_VERSION: u16 = 0xCA02;
    pub const PAYLOAD_SIZE: usize = 74;
    pub const TOTAL_SIZE: usize = 76;

    pub fn validate_finite(&self) -> Result<(), ProtocolError> {
        let h = self.mag_hard_iron;
        let g = self.gyro_bias_rps;
        let a = self.accel_scale;
        let base = [h[0], h[1], h[2], g[0], g[1], g[2], a[0], a[1], a[2]];
        for &v in &base {
            if !v.is_finite() {
                return Err(ProtocolError::NonFiniteFloat);
            }
        }
        for row in &self.mag_soft_iron {
            for &v in row {
                if !v.is_finite() {
                    return Err(ProtocolError::NonFiniteFloat);
                }
            }
        }
        Ok(())
    }

    pub fn default_calibrated() -> Self {
        Self {
            version: Self::MAGIC_VERSION,
            mag_hard_iron: [0.0; 3],
            mag_soft_iron: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            gyro_bias_rps: [0.0; 3],
            accel_scale: [1.0; 3],
        }
    }

    pub fn serialize(&self, dst: &mut [u8]) -> Result<usize, ProtocolError> {
        if dst.len() < Self::TOTAL_SIZE {
            return Err(ProtocolError::BufferTooSmall);
        }
        self.validate_finite()?;
        let mut idx = 0usize;
        dst[idx..idx + 2].copy_from_slice(&self.version.to_be_bytes());
        idx += 2;
        for &v in &self.mag_hard_iron {
            dst[idx..idx + 4].copy_from_slice(&v.to_be_bytes());
            idx += 4;
        }
        for row in &self.mag_soft_iron {
            for &v in row {
                dst[idx..idx + 4].copy_from_slice(&v.to_be_bytes());
                idx += 4;
            }
        }
        for &v in &self.gyro_bias_rps {
            dst[idx..idx + 4].copy_from_slice(&v.to_be_bytes());
            idx += 4;
        }
        for &v in &self.accel_scale {
            dst[idx..idx + 4].copy_from_slice(&v.to_be_bytes());
            idx += 4;
        }
        let crc = crc16_ccitt(&dst[..idx]);
        dst[idx..idx + 2].copy_from_slice(&crc.to_be_bytes());
        idx += 2;
        Ok(idx)
    }

    pub fn deserialize(src: &[u8]) -> Result<Self, ProtocolError> {
        if src.len() < Self::TOTAL_SIZE {
            return Err(ProtocolError::BufferTooSmall);
        }
        let payload = &src[..Self::PAYLOAD_SIZE];
        let expected_crc =
            u16::from_be_bytes([src[Self::PAYLOAD_SIZE], src[Self::PAYLOAD_SIZE + 1]]);
        if crc16_ccitt(payload) != expected_crc {
            return Err(ProtocolError::CrcMismatch);
        }
        let version = u16::from_be_bytes([payload[0], payload[1]]);
        if version != Self::MAGIC_VERSION {
            return Err(ProtocolError::VersionMismatch);
        }
        let mut idx = 2usize;
        Self::deserialize_fields(version, payload, &mut idx)
    }

    fn deserialize_fields(version: u16, p: &[u8], idx: &mut usize) -> Result<Self, ProtocolError> {
        let mut mag_hard_iron = [0.0f32; 3];
        for v in &mut mag_hard_iron {
            *v = f32::from_be_bytes([p[*idx], p[*idx + 1], p[*idx + 2], p[*idx + 3]]);
            *idx += 4;
        }
        let mut mag_soft_iron = [[0.0f32; 3]; 3];
        for row in &mut mag_soft_iron {
            for v in row {
                *v = f32::from_be_bytes([p[*idx], p[*idx + 1], p[*idx + 2], p[*idx + 3]]);
                *idx += 4;
            }
        }
        Self::deserialize_remainder(version, mag_hard_iron, mag_soft_iron, p, idx)
    }

    fn deserialize_remainder(
        version: u16,
        mag_hard_iron: [f32; 3],
        mag_soft_iron: [[f32; 3]; 3],
        p: &[u8],
        idx: &mut usize,
    ) -> Result<Self, ProtocolError> {
        let mut gyro_bias_rps = [0.0f32; 3];
        for v in &mut gyro_bias_rps {
            *v = f32::from_be_bytes([p[*idx], p[*idx + 1], p[*idx + 2], p[*idx + 3]]);
            *idx += 4;
        }
        let mut accel_scale = [0.0f32; 3];
        for v in &mut accel_scale {
            *v = f32::from_be_bytes([p[*idx], p[*idx + 1], p[*idx + 2], p[*idx + 3]]);
            *idx += 4;
        }
        let block = Self {
            version,
            mag_hard_iron,
            mag_soft_iron,
            gyro_bias_rps,
            accel_scale,
        };
        block.validate_finite()?;
        Ok(block)
    }
}
