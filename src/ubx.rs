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

impl UbxNavPvt {
    /// Decode a 92-byte NAV-PVT payload buffer without allocation.
    pub fn decode(payload: &[u8]) -> Result<Self, crate::protocol::ProtocolError> {
        if payload.len() < UBX_NAV_PVT_LEN {
            return Err(crate::protocol::ProtocolError::BufferTooSmall);
        }
        let i_tow_ms = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        let year = u16::from_le_bytes([payload[4], payload[5]]);
        let month = payload[6];
        let day = payload[7];
        let hour = payload[8];
        let min = payload[9];
        let sec = payload[10];
        let valid_flags = payload[11];
        let t_acc_ns = u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]);
        let nano_ns = i32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]);
        let fix_type = payload[20];
        let flags = payload[21];
        let flags2 = payload[22];
        let num_sv = payload[23];
        let lon_dege7 = i32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);
        let lat_dege7 = i32::from_le_bytes([payload[28], payload[29], payload[30], payload[31]]);
        let height_mm = i32::from_le_bytes([payload[32], payload[33], payload[34], payload[35]]);
        let h_msl_mm = i32::from_le_bytes([payload[36], payload[37], payload[38], payload[39]]);
        let h_acc_mm = u32::from_le_bytes([payload[40], payload[41], payload[42], payload[43]]);
        let v_acc_mm = u32::from_le_bytes([payload[44], payload[45], payload[46], payload[47]]);
        let vel_n_mmps = i32::from_le_bytes([payload[48], payload[49], payload[50], payload[51]]);
        let vel_e_mmps = i32::from_le_bytes([payload[52], payload[53], payload[54], payload[55]]);
        let vel_d_mmps = i32::from_le_bytes([payload[56], payload[57], payload[58], payload[59]]);
        let g_speed_mmps = i32::from_le_bytes([payload[60], payload[61], payload[62], payload[63]]);
        let head_mot_dege5 =
            i32::from_le_bytes([payload[64], payload[65], payload[66], payload[67]]);
        let s_acc_mmps = u32::from_le_bytes([payload[68], payload[69], payload[70], payload[71]]);
        let head_acc_dege5 =
            u32::from_le_bytes([payload[72], payload[73], payload[74], payload[75]]);
        let p_dop = u16::from_le_bytes([payload[76], payload[77]]);
        let flags3 = u16::from_le_bytes([payload[78], payload[79]]);
        let head_veh_dege5 =
            i32::from_le_bytes([payload[84], payload[85], payload[86], payload[87]]);
        Ok(Self {
            i_tow_ms,
            year,
            month,
            day,
            hour,
            min,
            sec,
            valid_flags,
            t_acc_ns,
            nano_ns,
            fix_type,
            flags,
            flags2,
            num_sv,
            lon_dege7,
            lat_dege7,
            height_mm,
            h_msl_mm,
            h_acc_mm,
            v_acc_mm,
            vel_n_mmps,
            vel_e_mmps,
            vel_d_mmps,
            g_speed_mmps,
            head_mot_dege5,
            s_acc_mmps,
            head_acc_dege5,
            p_dop,
            flags3,
            head_veh_dege5,
        })
    }
}

/// Decoded UBX-NAV-RELPOSNED relative positioning payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UbxNavRelPosNed {
    pub version: u8,
    pub station_id: u16,
    pub i_tow_ms: u32,
    pub rel_pos_n_cm: i32,
    pub rel_pos_e_cm: i32,
    pub rel_pos_d_cm: i32,
    pub rel_pos_length_cm: i32,
    pub rel_pos_heading_dege5: i32,
    pub rel_pos_hp_n_mm0_1: i8,
    pub rel_pos_hp_e_mm0_1: i8,
    pub rel_pos_hp_d_mm0_1: i8,
    pub rel_pos_hp_length_mm0_1: i8,
    pub acc_n_mm0_1: u32,
    pub acc_e_mm0_1: u32,
    pub acc_d_mm0_1: u32,
    pub acc_length_mm0_1: u32,
    pub acc_heading_dege5: u32,
    pub flags: u32,
}

impl UbxNavRelPosNed {
    /// Decode a 64-byte NAV-RELPOSNED payload buffer without allocation.
    pub fn decode(payload: &[u8]) -> Result<Self, crate::protocol::ProtocolError> {
        if payload.len() < UBX_NAV_RELPOSNED_LEN {
            return Err(crate::protocol::ProtocolError::BufferTooSmall);
        }
        let version = payload[0];
        let station_id = u16::from_le_bytes([payload[2], payload[3]]);
        let i_tow_ms = u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        let rel_pos_n_cm = i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);
        let rel_pos_e_cm = i32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]);
        let rel_pos_d_cm = i32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]);
        let rel_pos_length_cm =
            i32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]);
        let rel_pos_heading_dege5 =
            i32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);
        let rel_pos_hp_n_mm0_1 = payload[32] as i8;
        let rel_pos_hp_e_mm0_1 = payload[33] as i8;
        let rel_pos_hp_d_mm0_1 = payload[34] as i8;
        let rel_pos_hp_length_mm0_1 = payload[35] as i8;
        let acc_n_mm0_1 = u32::from_le_bytes([payload[36], payload[37], payload[38], payload[39]]);
        let acc_e_mm0_1 = u32::from_le_bytes([payload[40], payload[41], payload[42], payload[43]]);
        let acc_d_mm0_1 = u32::from_le_bytes([payload[44], payload[45], payload[46], payload[47]]);
        let acc_length_mm0_1 =
            u32::from_le_bytes([payload[48], payload[49], payload[50], payload[51]]);
        let acc_heading_dege5 =
            u32::from_le_bytes([payload[52], payload[53], payload[54], payload[55]]);
        let flags = u32::from_le_bytes([payload[60], payload[61], payload[62], payload[63]]);
        Ok(Self {
            version,
            station_id,
            i_tow_ms,
            rel_pos_n_cm,
            rel_pos_e_cm,
            rel_pos_d_cm,
            rel_pos_length_cm,
            rel_pos_heading_dege5,
            rel_pos_hp_n_mm0_1,
            rel_pos_hp_e_mm0_1,
            rel_pos_hp_d_mm0_1,
            rel_pos_hp_length_mm0_1,
            acc_n_mm0_1,
            acc_e_mm0_1,
            acc_d_mm0_1,
            acc_length_mm0_1,
            acc_heading_dege5,
            flags,
        })
    }
}

/// Decoded UBX message variant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UbxMessage {
    NavPvt(UbxNavPvt),
    NavRelPosNed(UbxNavRelPosNed),
}

/// Zero-alloc streaming UBX framing parser.
pub struct UbxParser {
    buf: [u8; 128],
    len: usize,
}
