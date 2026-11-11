//! Zero-alloc protocol codec and telemetry framing.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
    BufferTooSmall,
    InvalidCobsEncoding,
    CrcMismatch,
    InvalidCanId,
    InvalidCanDlc,
    NonFiniteFloat,
}

pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

pub fn cobs_encode(src: &[u8], dst: &mut [u8]) -> Result<usize, ProtocolError> {
    if dst.is_empty() {
        return Err(ProtocolError::BufferTooSmall);
    }
    let mut code_idx = 0usize;
    let mut write_idx = 1usize;
    let mut code = 1u8;

    for (i, &byte) in src.iter().enumerate() {
        if byte == 0 {
            dst[code_idx] = code;
            code_idx = write_idx;
            if write_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            write_idx += 1;
            code = 1;
        } else {
            if write_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            dst[write_idx] = byte;
            write_idx += 1;
            code += 1;
            if code == 0xFF {
                dst[code_idx] = code;
                if i + 1 < src.len() {
                    code_idx = write_idx;
                    if write_idx >= dst.len() {
                        return Err(ProtocolError::BufferTooSmall);
                    }
                    write_idx += 1;
                    code = 1;
                } else {
                    code = 0;
                }
            }
        }
    }
    if code != 0 {
        dst[code_idx] = code;
    }
    Ok(write_idx)
}

pub fn cobs_decode(src: &[u8], dst: &mut [u8]) -> Result<usize, ProtocolError> {
    if src.is_empty() {
        return Err(ProtocolError::InvalidCobsEncoding);
    }
    let mut src_idx = 0usize;
    let mut dst_idx = 0usize;

    while src_idx < src.len() {
        let code = src[src_idx];
        if code == 0 {
            return Err(ProtocolError::InvalidCobsEncoding);
        }
        src_idx += 1;
        for _ in 1..code {
            if src_idx >= src.len() {
                return Err(ProtocolError::InvalidCobsEncoding);
            }
            let byte = src[src_idx];
            if byte == 0 {
                return Err(ProtocolError::InvalidCobsEncoding);
            }
            if dst_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            dst[dst_idx] = byte;
            dst_idx += 1;
            src_idx += 1;
        }
        if code < 0xFF && src_idx < src.len() {
            if dst_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            dst[dst_idx] = 0;
            dst_idx += 1;
        }
    }
    Ok(dst_idx)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame20B {
    pub id: u32,
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CanFrame20B {
    pub const MAX_EXT_ID: u32 = 0x1FFF_FFFF;
    pub const MAX_DLC: u8 = 8;

    pub fn new(id: u32, dlc: u8, data: [u8; 8]) -> Result<Self, ProtocolError> {
        if id > Self::MAX_EXT_ID {
            return Err(ProtocolError::InvalidCanId);
        }
        if dlc > Self::MAX_DLC {
            return Err(ProtocolError::InvalidCanDlc);
        }
        Ok(Self { id, dlc, data })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavTelemetry {
    pub timestamp_ms: u32,
    pub pos_ned_m: [f32; 3],
    pub vel_ned_mps: [f32; 3],
    pub euler_rad: [f32; 3],
    pub status: u8,
}

impl NavTelemetry {
    pub const RAW_SIZE: usize = 41;
    pub const MAX_UART_FRAME_LEN: usize = 64;

    pub fn validate_finite(&self) -> Result<(), ProtocolError> {
        let all_floats = [
            self.pos_ned_m[0],
            self.pos_ned_m[1],
            self.pos_ned_m[2],
            self.vel_ned_mps[0],
            self.vel_ned_mps[1],
            self.vel_ned_mps[2],
            self.euler_rad[0],
            self.euler_rad[1],
            self.euler_rad[2],
        ];
        for val in all_floats {
            if !val.is_finite() {
                return Err(ProtocolError::NonFiniteFloat);
            }
        }
        Ok(())
    }

    pub fn serialize_raw(&self, out: &mut [u8; Self::RAW_SIZE]) -> Result<(), ProtocolError> {
        self.validate_finite()?;
        out[0..4].copy_from_slice(&self.timestamp_ms.to_be_bytes());
        for (i, &pos_val) in self.pos_ned_m.iter().enumerate() {
            let s = 4 + i * 4;
            out[s..s + 4].copy_from_slice(&pos_val.to_bits().to_be_bytes());
        }
        for (i, &vel_val) in self.vel_ned_mps.iter().enumerate() {
            let s = 16 + i * 4;
            out[s..s + 4].copy_from_slice(&vel_val.to_bits().to_be_bytes());
        }
        for (i, &euler_val) in self.euler_rad.iter().enumerate() {
            let s = 28 + i * 4;
            out[s..s + 4].copy_from_slice(&euler_val.to_bits().to_be_bytes());
        }
        out[40] = self.status;
        Ok(())
    }

    pub fn deserialize_raw(buf: &[u8; Self::RAW_SIZE]) -> Result<Self, ProtocolError> {
        let timestamp_ms = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let mut pos = [0.0f32; 3];
        for (i, slot) in pos.iter_mut().enumerate() {
            let s = 4 + i * 4;
            let bits = u32::from_be_bytes([buf[s], buf[s + 1], buf[s + 2], buf[s + 3]]);
            *slot = f32::from_bits(bits);
        }
        let mut vel = [0.0f32; 3];
        for (i, slot) in vel.iter_mut().enumerate() {
            let s = 16 + i * 4;
            let bits = u32::from_be_bytes([buf[s], buf[s + 1], buf[s + 2], buf[s + 3]]);
            *slot = f32::from_bits(bits);
        }
        let mut euler = [0.0f32; 3];
        for (i, slot) in euler.iter_mut().enumerate() {
            let s = 28 + i * 4;
            let bits = u32::from_be_bytes([buf[s], buf[s + 1], buf[s + 2], buf[s + 3]]);
            *slot = f32::from_bits(bits);
        }
        let telem = Self {
            timestamp_ms,
            pos_ned_m: pos,
            vel_ned_mps: vel,
            euler_rad: euler,
            status: buf[40],
        };
        telem.validate_finite()?;
        Ok(telem)
    }

    pub fn pack_uart(&self, dst: &mut [u8]) -> Result<usize, ProtocolError> {
        let mut payload = [0u8; Self::RAW_SIZE + 2];
        let mut raw = [0u8; Self::RAW_SIZE];
        self.serialize_raw(&mut raw)?;
