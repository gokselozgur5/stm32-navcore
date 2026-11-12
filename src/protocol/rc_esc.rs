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

/// CRSF protocol decoding errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrsfError {
    /// Buffer is smaller than minimum required frame length.
    BufferTooShort,
    /// Header sync byte does not match 0xC8.
    InvalidSyncByte,
    /// Length field does not match expected payload size.
    InvalidLength,
    /// Unknown or unsupported frame type.
    UnsupportedFrameType,
    /// CRC8 checksum mismatch.
    ChecksumMismatch,
    /// DShot checksum error.
    DshotChecksumError,
}

/// Zero-allocation stream decoder for CRSF frames.
#[derive(Debug, Clone)]
pub struct CrsfParser {
    buffer: [u8; 64],
    len: usize,
}

impl Default for CrsfParser {
    fn default() -> Self {
        Self::new()
    }
}

impl CrsfParser {
    /// Create a new CRSF stream parser with empty internal buffer.
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; 64],
            len: 0,
        }
    }

    /// Reset parser state and clear internal buffer.
    pub fn reset(&mut self) {
        self.len = 0;
    }
}

impl CrsfParser {
    /// Unpack 16 11-bit channels from 22-byte CRSF payload.
    pub fn unpack_channels(payload: &[u8; 22]) -> [u16; 16] {
        let mut ch = [0u16; 16];
        let mut bit_idx = 0usize;
        let mut i = 0;
        while i < 16 {
            let mut val = 0u16;
            let mut b = 0;
            while b < 11 {
                let byte_pos = bit_idx / 8;
                let bit_pos = bit_idx % 8;
                if (payload[byte_pos] & (1 << bit_pos)) != 0 {
                    val |= 1 << b;
                }
                bit_idx += 1;
                b += 1;
            }
            ch[i] = val;
            i += 1;
        }
        ch
    }
}

impl CrsfParser {
    fn validate_header(frame: &[u8]) -> Result<usize, CrsfError> {
        if frame.len() < 4 {
            return Err(CrsfError::BufferTooShort);
        }
        if frame[0] != CRSF_SYNC_BYTE {
            return Err(CrsfError::InvalidSyncByte);
        }
        let len = frame[1] as usize;
        if frame.len() < len + 2 {
            return Err(CrsfError::BufferTooShort);
        }
        if frame[2] != CRSF_FRAME_TYPE_RC_CHANNELS {
            return Err(CrsfError::UnsupportedFrameType);
        }
        if len != 24 {
            return Err(CrsfError::InvalidLength);
        }
        Ok(len)
    }
}

impl CrsfParser {
    /// Parse a complete CRSF frame from a byte slice.
    pub fn parse_frame(frame: &[u8]) -> Result<CrsfFrame, CrsfError> {
        let _ = Self::validate_header(frame)?;
        let expected_crc = frame[25];
        let calc_crc = crc8_dvb_s2(&frame[2..25]);
        if expected_crc != calc_crc {
            return Err(CrsfError::ChecksumMismatch);
        }
        let mut payload = [0u8; 22];
        payload.copy_from_slice(&frame[3..25]);
        let raw_channels = Self::unpack_channels(&payload);
        Ok(CrsfFrame { raw_channels })
    }
}

impl CrsfParser {
    /// Push a single byte into the stream parser; returns frame when complete.
    pub fn push_byte(&mut self, byte: u8) -> Result<Option<CrsfFrame>, CrsfError> {
        if self.len == 0 {
            if byte == CRSF_SYNC_BYTE {
                self.buffer[0] = byte;
                self.len = 1;
            }
            return Ok(None);
        }
        if self.len < self.buffer.len() {
            self.buffer[self.len] = byte;
            self.len += 1;
        } else {
            self.reset();
            return Err(CrsfError::BufferTooShort);
        }
        if self.len >= 2 {
            let expected_total = (self.buffer[1] as usize) + 2;
            if self.len == expected_total {
                let res = Self::parse_frame(&self.buffer[..self.len]);
                self.reset();
                return res.map(Some);
            }
        }
        Ok(None)
    }
}

/// DShot digital ESC protocol speed standard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DshotSpeed {
    /// DShot150 (150 kbit/s, bit time 6.67 us).
    Dshot150,
    /// DShot300 (300 kbit/s, bit time 3.33 us).
    Dshot300,
    /// DShot600 (600 kbit/s, bit time 1.67 us).
    Dshot600,
}

impl DshotSpeed {
    /// Return nominal bit duration in nanoseconds.
    #[inline]
    pub const fn bit_period_ns(&self) -> u32 {
        match self {
            Self::Dshot150 => 6667,
            Self::Dshot300 => 3333,
            Self::Dshot600 => 1667,
        }
    }
}

/// 16-bit DShot command frame (11-bit throttle, 1-bit telemetry, 4-bit CRC).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DshotFrame {
    /// 11-bit throttle or command value (0..2047).
    pub throttle: u16,
    /// Telemetry request flag bit.
    pub telemetry: bool,
}
