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
