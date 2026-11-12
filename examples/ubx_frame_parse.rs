//! UBX-NAV-PVT frame construction and streaming parse.
//!
//! Builds a checksummed UBX-NAV-PVT frame, feeds it to `UbxParser` one byte
//! at a time, and checks the decoded fields against the input values.
//!
//! Run with `cargo run --example ubx_frame_parse`.

use stm32_navcore::ubx::{
    ubx_checksum, UbxMessage, UbxNavPvt, UbxParser, UBX_CLASS_NAV, UBX_ID_NAV_PVT,
    UBX_NAV_PVT_LEN, UBX_SYNC_1, UBX_SYNC_2,
};

/// GPS time of week written into the test frame, in milliseconds.
const TEST_ITOW_MS: u32 = 123_456_789;
/// Latitude in 1e-7 degrees (about 52.52 N).
const TEST_LAT_DEGE7: i32 = 525_200_000;
/// Longitude in 1e-7 degrees (about 13.405 E).
const TEST_LON_DEGE7: i32 = 134_050_000;
/// Full frame size: sync (2) + class and id (2) + length (2) + payload + checksum (2).
const FRAME_LEN: usize = 8 + UBX_NAV_PVT_LEN;

/// Fills a NAV-PVT payload with a fixed time, date, fix type, and position.
///
/// Offsets follow the u-blox interface description for UBX-NAV-PVT.
fn build_pvt_payload(p: &mut [u8; UBX_NAV_PVT_LEN]) {
    p[0..4].copy_from_slice(&TEST_ITOW_MS.to_le_bytes());
    p[4..6].copy_from_slice(&2026u16.to_le_bytes());
    p[6] = 11; // month
    p[7] = 12; // day
    p[20] = 3; // fixType: 3D fix
    p[23] = 14; // numSV
    p[24..28].copy_from_slice(&TEST_LON_DEGE7.to_le_bytes());
    p[28..32].copy_from_slice(&TEST_LAT_DEGE7.to_le_bytes());
}

/// Wraps `payload` in a UBX frame: sync bytes, class, id, length, checksum.
///
/// Returns the number of bytes written to `out`.
fn build_frame(class: u8, id: u8, payload: &[u8], out: &mut [u8]) -> usize {
    let len = payload.len();
    out[0] = UBX_SYNC_1;
    out[1] = UBX_SYNC_2;
    out[2] = class;
    out[3] = id;
    out[4..6].copy_from_slice(&(len as u16).to_le_bytes());
    out[6..6 + len].copy_from_slice(payload);
    // The Fletcher checksum covers class, id, length, and payload.
    let (ck_a, ck_b) = ubx_checksum(&out[2..6 + len]);
    out[6 + len] = ck_a;
    out[7 + len] = ck_b;
    8 + len
}

/// Feeds a byte stream to the parser and returns the first NAV-PVT found.
///
/// Parser errors are fatal here because the input is generated locally.
fn parse_stream(stream: &[u8]) -> Option<UbxNavPvt> {
    let mut parser = UbxParser::new();
    for &byte in stream {
        match parser.process_byte(byte).expect("frame rejected") {
            Some(UbxMessage::NavPvt(pvt)) => return Some(pvt),
            Some(other) => println!("ignoring {other:?}"),
            None => {}
        }
    }
    None
}
