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
