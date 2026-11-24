pub mod rc_esc;
pub use rc_esc::{CrsfFrame, CrsfParser, DshotFrame, DshotSpeed};
pub mod calibration;
pub use calibration::CalibrationBlock;
pub mod codec;
pub use codec::{cobs_decode, cobs_encode, crc16_ccitt};
pub mod telemetry;
pub use telemetry::{CanFrame20B, NavTelemetry};
pub mod ubx;
pub use ubx::{ubx_checksum, UbxMessage, UbxNavPvt, UbxNavRelPosNed, UbxParser};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
    BufferTooSmall,
    InvalidCobsEncoding,
    CrcMismatch,
    InvalidCanId,
    InvalidCanDlc,
    NonFiniteFloat,
    VersionMismatch,
    CorruptedData,
}
