pub use crate::protocol::ProtocolError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame20B {
    pub id: u32,
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CanFrame20B {
    pub const MAX_EXT_ID: u32 = 0x1FFF_FFFF;
    pub const MAX_DLC: u8 = 8;

    pub fn new(id: u32, dlc: u8, data: [u8; 8]) -> Result<Self, crate::protocol::ProtocolError> {
        if id > Self::MAX_EXT_ID {
            return Err(crate::protocol::ProtocolError::InvalidCanId);
        }
        if dlc > Self::MAX_DLC {
            return Err(crate::protocol::ProtocolError::InvalidCanDlc);
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
    // ... I'll copy the NavTelemetry impl ...
}
// ... and the CRC/COBS functions ...
