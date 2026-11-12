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
