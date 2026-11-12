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
