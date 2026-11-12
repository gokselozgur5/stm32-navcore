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

/// Compute 8-bit Fletcher checksum over slice (CK_A, CK_B).
#[inline]
pub fn ubx_checksum(bytes: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in bytes {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Decoded UBX-NAV-PVT payload data representation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UbxNavPvt {
    pub i_tow_ms: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid_flags: u8,
    pub t_acc_ns: u32,
    pub nano_ns: i32,
    pub fix_type: u8,
    pub flags: u8,
    pub flags2: u8,
    pub num_sv: u8,
    pub lon_dege7: i32,
    pub lat_dege7: i32,
    pub height_mm: i32,
    pub h_msl_mm: i32,
    pub h_acc_mm: u32,
    pub v_acc_mm: u32,
    pub vel_n_mmps: i32,
    pub vel_e_mmps: i32,
    pub vel_d_mmps: i32,
    pub g_speed_mmps: i32,
    pub head_mot_dege5: i32,
    pub s_acc_mmps: u32,
    pub head_acc_dege5: u32,
    pub p_dop: u16,
    pub flags3: u16,
    pub head_veh_dege5: i32,
}
