//! COBS framing roundtrip with a CRC-16 trailer.
//!
//! Encodes a payload with `cobs_encode`, appends a CRC-16-CCITT check value,
//! decodes the frame with `cobs_decode`, and verifies the payload byte by byte.
//!
//! Run with `cargo run --example cobs_roundtrip`.

use stm32_navcore::protocol::{cobs_decode, cobs_encode, crc16_ccitt, ProtocolError};

/// Payload with embedded zero bytes, which COBS must escape.
const PAYLOAD: [u8; 8] = [0x11, 0x00, 0x22, 0x33, 0x00, 0x00, 0x44, 0x55];

/// Builds a wire frame: COBS(payload || CRC-16 big endian), then a 0x00 delimiter.
///
/// Returns the total number of bytes written to `out`, delimiter included.
fn encode_frame(payload: &[u8], out: &mut [u8]) -> Result<usize, ProtocolError> {
    let mut raw = [0u8; 32];
    raw[..payload.len()].copy_from_slice(payload);
    let crc = crc16_ccitt(payload).to_be_bytes();
    raw[payload.len()..payload.len() + 2].copy_from_slice(&crc);
    let n = cobs_encode(&raw[..payload.len() + 2], out)?;
    *out.get_mut(n).ok_or(ProtocolError::BufferTooSmall)? = 0x00;
    Ok(n + 1)
}
