//! Zero-alloc u-blox Binary UBX Protocol Engine.
//!
//! Supports streaming synchronization and bit-exact extraction for
//! UBX-NAV-PVT (0x01 0x07) and UBX-NAV-RELPOSNED (0x01 0x3C).

pub const UBX_SYNC_1: u8 = 0xB5;
pub const UBX_SYNC_2: u8 = 0x62;
pub const UBX_CLASS_NAV: u8 = 0x01;
pub const UBX_ID_NAV_PVT: u8 = 0x07;
pub const UBX_ID_NAV_RELPOSNED: u8 = 0x3C;
pub const UBX_NAV_PVT_LEN: usize = 92;
pub const UBX_NAV_RELPOSNED_LEN: usize = 64;
