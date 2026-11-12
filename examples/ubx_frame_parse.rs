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
