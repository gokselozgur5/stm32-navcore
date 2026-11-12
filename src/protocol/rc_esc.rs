//! CRSF RC Receiver Telemetry Decoder & DShot300/600 Digital Motor Codec.
//! Zero-alloc decoding for TBS Crossfire / ELRS and DShot ESC commands.

/// CRSF synchronization byte.
pub const CRSF_SYNC_BYTE: u8 = 0xC8;

/// CRSF 16-channel RC data frame type.
pub const CRSF_FRAME_TYPE_RC_CHANNELS: u8 = 0x16;

/// Minimum CRSF channel raw integer value.
pub const CRSF_CHANNEL_MIN: u16 = 172;

/// Center CRSF channel raw integer value.
pub const CRSF_CHANNEL_MID: u16 = 992;

/// Maximum CRSF channel raw integer value.
pub const CRSF_CHANNEL_MAX: u16 = 1811;

/// Compute CRC8 DVB-S2 for CRSF frame validation (polynomial 0xD5).
#[inline]
pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    let mut i = 0;
    while i < data.len() {
        crc ^= data[i];
        let mut bit = 0;
        while bit < 8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    crc
}

/// Decoded CRSF 16-channel RC frame with raw and normalized channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrsfFrame {
    /// 16 analog channels in raw 11-bit integer form (172..1811).
    pub raw_channels: [u16; 16],
}

impl Default for CrsfFrame {
    fn default() -> Self {
        Self {
            raw_channels: [CRSF_CHANNEL_MID; 16],
        }
    }
}

impl CrsfFrame {
    /// Return normalized channel value in [-1.0, 1.0] range.
    #[inline]
    pub fn normalized_channel(&self, index: usize) -> f32 {
        if index >= 16 {
            return 0.0;
        }
        let raw = self.raw_channels[index];
        if raw <= CRSF_CHANNEL_MID {
            let span = (CRSF_CHANNEL_MID - CRSF_CHANNEL_MIN) as f32;
            ((raw as f32) - (CRSF_CHANNEL_MID as f32)) / span
        } else {
            let span = (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID) as f32;
            ((raw as f32) - (CRSF_CHANNEL_MID as f32)) / span
        }
    }
}
