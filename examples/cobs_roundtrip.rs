//! COBS framing roundtrip with a CRC-16 trailer.
//!
//! Encodes a payload with `cobs_encode`, appends a CRC-16-CCITT check value,
//! decodes the frame with `cobs_decode`, and verifies the payload byte by byte.
//!
//! Run with `cargo run --example cobs_roundtrip`.

use stm32_navcore::protocol::{cobs_decode, cobs_encode, crc16_ccitt, ProtocolError};

/// Payload with embedded zero bytes, which COBS must escape.
const PAYLOAD: [u8; 8] = [0x11, 0x00, 0x22, 0x33, 0x00, 0x00, 0x44, 0x55];
