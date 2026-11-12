//! CRSF RC Receiver Telemetry Decoder & DShot Digital Motor Codec.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RcEscError {
    InvalidSyncByte,
    InvalidFrameLength,
    InvalidFrameType,
    CrcMismatch,
    BufferTooSmall,
    InvalidThrottleValue,
}

pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

pub const CRSF_SYNC_BYTE: u8 = 0xC8;
pub const CRSF_FRAME_CHANNELS: u8 = 0x16;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrsfChannels {
    pub channels: [f32; 16],
}

impl CrsfChannels {
    fn unpack_single_channel(raw_22: &[u8; 22], bit_pos: usize) -> u16 {
        let byte_idx = bit_pos / 8;
        let bit_off = bit_pos % 8;
        let mut val = (raw_22[byte_idx] as u32) >> bit_off;
        if bit_off > 5 && byte_idx + 1 < 22 {
            val |= (raw_22[byte_idx + 1] as u32) << (8 - bit_off);
        }
        if bit_off > 13 && byte_idx + 2 < 22 {
            val |= (raw_22[byte_idx + 2] as u32) << (16 - bit_off);
        }
        (val & 0x07FF) as u16
    }
}

impl CrsfChannels {
    pub fn unpack_channels(raw_22: &[u8; 22]) -> [u16; 16] {
        let mut ch = [0u16; 16];
        let mut i = 0;
        while i < 16 {
            ch[i] = Self::unpack_single_channel(raw_22, i * 11);
            i += 1;
        }
        ch
    }
}

impl CrsfChannels {
    pub fn normalize_channel(raw: u16) -> f32 {
        let clamped = if raw < 172 {
            172
        } else if raw > 1811 {
            1811
        } else {
            raw
        };
        (clamped as f32 - 992.0) / 819.0
    }
}

impl CrsfChannels {
    fn check_header(buf: &[u8]) -> Result<(), RcEscError> {
        if buf.len() < 26 {
            return Err(RcEscError::BufferTooSmall);
        }
        if buf[0] != CRSF_SYNC_BYTE {
            return Err(RcEscError::InvalidSyncByte);
        }
        if buf[1] != 24 {
            return Err(RcEscError::InvalidFrameLength);
        }
        if buf[2] != CRSF_FRAME_CHANNELS {
            return Err(RcEscError::InvalidFrameType);
        }
        Ok(())
    }
}
