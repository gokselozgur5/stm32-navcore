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

/// RTK carrier-phase solution state reported by the receiver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarrierSolution {
    None,
    Float,
    Fixed,
}

impl CarrierSolution {
    /// Map the 2-bit `carrSoln` field; the reserved value 3 maps to `None`.
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            1 => Self::Float,
            2 => Self::Fixed,
            _ => Self::None,
        }
    }
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

impl UbxNavPvt {
    /// Carrier solution from `flags` bits 6..7.
    pub const fn carrier_solution(&self) -> CarrierSolution {
        CarrierSolution::from_bits(self.flags >> 6)
    }

    /// `gnssFixOK` flag from `flags` bit 0.
    pub const fn gnss_fix_ok(&self) -> bool {
        self.flags & 0x01 != 0
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

impl Default for UbxParser {
    fn default() -> Self {
        Self::new()
    }
}

impl UbxParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 128],
            len: 0,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Total frame length declared by a buffered header.
    fn frame_len(buf: &[u8; 128]) -> usize {
        8 + u16::from_le_bytes([buf[4], buf[5]]) as usize
    }

    /// Drop `n` buffered bytes, then keep bytes from the next plausible sync.
    fn discard(&mut self, mut n: usize) {
        loop {
            let len = self.len;
            let buf = &self.buf;
            let start = (n..len)
                .find(|&i| buf[i] == UBX_SYNC_1 && (i + 1 == len || buf[i + 1] == UBX_SYNC_2))
                .unwrap_or(len);
            self.buf.copy_within(start..len, 0);
            self.len = len - start;
            if self.len < 6 || Self::frame_len(&self.buf) <= self.buf.len() {
                return;
            }
            n = 1;
        }
    }

    pub fn process_byte(
        &mut self,
        b: u8,
    ) -> Result<Option<UbxMessage>, crate::protocol::ProtocolError> {
        if self.len == 0 {
            if b == UBX_SYNC_1 {
                self.buf[0] = b;
                self.len = 1;
            }
            return Ok(None);
        }
        if self.len == 1 {
            if b == UBX_SYNC_2 {
                self.buf[1] = b;
                self.len = 2;
            } else if b != UBX_SYNC_1 {
                self.len = 0;
            }
            return Ok(None);
        }
        self.buf[self.len] = b;
        self.len += 1;
        if self.len < 6 {
            return Ok(None);
        }
        let payload_len = u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;
        let total_expected = 6 + payload_len + 2;
        if total_expected > self.buf.len() {
            self.discard(1);
            return Err(crate::protocol::ProtocolError::BufferTooSmall);
        }
        if self.len < total_expected {
            return Ok(None);
        }
        let msg_class = self.buf[2];
        let msg_id = self.buf[3];
        let (calc_a, calc_b) = ubx_checksum(&self.buf[2..6 + payload_len]);
        let (rx_a, rx_b) = (self.buf[6 + payload_len], self.buf[6 + payload_len + 1]);
        if calc_a != rx_a || calc_b != rx_b {
            self.discard(1);
            return Err(crate::protocol::ProtocolError::CrcMismatch);
        }
        let payload = &self.buf[6..6 + payload_len];
        let msg = if msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_NAV_PVT {
            UbxNavPvt::decode(payload).map(|m| Some(UbxMessage::NavPvt(m)))
        } else if msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_NAV_RELPOSNED {
            UbxNavRelPosNed::decode(payload).map(|m| Some(UbxMessage::NavRelPosNed(m)))
        } else {
            Ok(None)
        };
        self.discard(total_expected);
        msg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ubx_checksum_vector() {
        let msg = [0x01, 0x07, 0x00, 0x00];
        let (ck_a, ck_b) = ubx_checksum(&msg);
        assert_eq!(ck_a, 0x08);
        assert_eq!(ck_b, 0x19);
    }

    #[test]
    fn test_ubx_parser_recovers_after_repeated_sync_byte() {
        let mut frame = [0u8; 6 + 64 + 2];
        frame[0] = UBX_SYNC_1;
        frame[1] = UBX_SYNC_2;
        frame[2] = UBX_CLASS_NAV;
        frame[3] = UBX_ID_NAV_RELPOSNED;
        frame[4..6].copy_from_slice(&(64u16).to_le_bytes());
        let (ck_a, ck_b) = ubx_checksum(&frame[2..6 + 64]);
        frame[70] = ck_a;
        frame[71] = ck_b;
        let mut parser = UbxParser::new();
        assert_eq!(parser.process_byte(UBX_SYNC_1).unwrap(), None);
        let mut res = None;
        for &b in &frame {
            if let Some(msg) = parser.process_byte(b).unwrap() {
                res = Some(msg);
            }
        }
        assert!(matches!(res, Some(UbxMessage::NavRelPosNed(_))));
    }

    #[test]
    fn test_ubx_nav_pvt_decode_and_fields() {
        let mut p = [0u8; 92];
        p[0..4].copy_from_slice(&123456789u32.to_le_bytes());
        p[4..6].copy_from_slice(&2026u16.to_le_bytes());
        p[6] = 11;
        p[7] = 12;
        p[20] = 3; // 3D-Fix
        p[24..28].copy_from_slice(&(-1224194000i32).to_le_bytes());
        p[28..32].copy_from_slice(&(377749000i32).to_le_bytes());
        let pvt = UbxNavPvt::decode(&p).expect("valid PVT");
        assert_eq!(pvt.i_tow_ms, 123456789);
        assert_eq!(pvt.year, 2026);
        assert_eq!(pvt.month, 11);
        assert_eq!(pvt.day, 12);
        assert_eq!(pvt.fix_type, 3);
        assert_eq!(pvt.lat_dege7, 377749000);
    }

    fn relposned_frame() -> [u8; 72] {
        let mut frame = [0u8; 72];
        frame[0] = UBX_SYNC_1;
        frame[1] = UBX_SYNC_2;
        frame[2] = UBX_CLASS_NAV;
        frame[3] = UBX_ID_NAV_RELPOSNED;
        frame[4..6].copy_from_slice(&(64u16).to_le_bytes());
        let (ck_a, ck_b) = ubx_checksum(&frame[2..70]);
        frame[70] = ck_a;
        frame[71] = ck_b;
        frame
    }

    fn count_relposned(parser: &mut UbxParser, bytes: &[u8]) -> usize {
        bytes
            .iter()
            .filter(|&&b| {
                matches!(
                    parser.process_byte(b),
                    Ok(Some(UbxMessage::NavRelPosNed(_)))
                )
            })
            .count()
    }

    #[test]
    fn test_ubx_parser_resyncs_after_truncated_frame() {
        let frame = relposned_frame();
        let mut stream = [0u8; 30 + 72];
        stream[..30].copy_from_slice(&frame[..30]);
        stream[30..].copy_from_slice(&frame);
        let mut parser = UbxParser::new();
        assert_eq!(count_relposned(&mut parser, &stream), 1);
    }

    #[test]
    fn test_ubx_parser_resyncs_after_false_sync_pair() {
        let frame = relposned_frame();
        let mut stream = [0u8; 2 + 72];
        stream[..2].copy_from_slice(&[UBX_SYNC_1, UBX_SYNC_2]);
        stream[2..].copy_from_slice(&frame);
        let mut parser = UbxParser::new();
        assert_eq!(count_relposned(&mut parser, &stream), 1);
    }

    #[test]
    fn test_ubx_streaming_parser_with_noise_and_crc_rejection() {
        let mut parser = UbxParser::new();
        let noise = [0xFF, 0x00, 0xAA, 0xB5, 0x11, 0x62];
        for &b in &noise {
            assert_eq!(parser.process_byte(b).unwrap(), None);
        }
        let mut frame = [0u8; 6 + 64 + 2];
        frame[0] = UBX_SYNC_1;
        frame[1] = UBX_SYNC_2;
        frame[2] = UBX_CLASS_NAV;
        frame[3] = UBX_ID_NAV_RELPOSNED;
        frame[4..6].copy_from_slice(&(64u16).to_le_bytes());
        frame[6 + 8..6 + 12].copy_from_slice(&(1500i32).to_le_bytes()); // rel_pos_n = 15m
        let (ck_a, ck_b) = ubx_checksum(&frame[2..6 + 64]);
        frame[6 + 64] = ck_a;
        frame[6 + 64 + 1] = ck_b;
        let mut res = None;
        for &b in &frame {
            if let Some(msg) = parser.process_byte(b).unwrap() {
                res = Some(msg);
            }
        }
        assert!(matches!(res, Some(UbxMessage::NavRelPosNed(_))));
    }
}

#[cfg(test)]
mod corruption_tests {
    use super::*;

    fn frame(class: u8, id: u8, payload_len: usize) -> ([u8; 128], usize) {
        let mut f = [0u8; 128];
        f[..4].copy_from_slice(&[UBX_SYNC_1, UBX_SYNC_2, class, id]);
        f[4..6].copy_from_slice(&(payload_len as u16).to_le_bytes());
        f[10] = 0x5A;
        let (ck_a, ck_b) = ubx_checksum(&f[2..6 + payload_len]);
        f[6 + payload_len] = ck_a;
        f[7 + payload_len] = ck_b;
        (f, 8 + payload_len)
    }

    fn count_messages(parser: &mut UbxParser, bytes: &[u8]) -> usize {
        bytes
            .iter()
            .filter(|&&b| matches!(parser.process_byte(b), Ok(Some(_))))
            .count()
    }

    #[test]
    fn every_single_bit_flip_is_rejected() {
        let (good, n) = frame(UBX_CLASS_NAV, UBX_ID_NAV_PVT, UBX_NAV_PVT_LEN);
        assert_eq!(count_messages(&mut UbxParser::new(), &good[..n]), 1);
        for bit in 0..n * 8 {
            let mut bad = good;
            bad[bit / 8] ^= 1 << (bit % 8);
            let mut parser = UbxParser::new();
            assert_eq!(count_messages(&mut parser, &bad[..n]), 0, "bit {bit}");
        }
    }

    #[test]
    fn oversized_length_is_rejected_then_recovers() {
        let mut parser = UbxParser::new();
        let header = [
            UBX_SYNC_1,
            UBX_SYNC_2,
            UBX_CLASS_NAV,
            UBX_ID_NAV_PVT,
            0xFF,
            0xFF,
        ];
        for &b in &header[..5] {
            assert_eq!(parser.process_byte(b), Ok(None));
        }
        assert_eq!(
            parser.process_byte(header[5]),
            Err(crate::protocol::ProtocolError::BufferTooSmall)
        );
        let (good, n) = frame(UBX_CLASS_NAV, UBX_ID_NAV_PVT, UBX_NAV_PVT_LEN);
        assert_eq!(count_messages(&mut parser, &good[..n]), 1);
    }

    #[test]
    fn pseudo_random_garbage_never_panics_or_emits() {
        let mut parser = UbxParser::new();
        let mut state: u32 = 0x1234_5678;
        for _ in 0..200_000 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let b = (state >> 24) as u8;
            assert!(!matches!(parser.process_byte(b), Ok(Some(_))));
        }
        parser.reset();
        let (good, n) = frame(UBX_CLASS_NAV, UBX_ID_NAV_RELPOSNED, UBX_NAV_RELPOSNED_LEN);
        assert_eq!(count_messages(&mut parser, &good[..n]), 1);
    }

    #[test]
    fn unsupported_message_is_consumed_silently() {
        let mut parser = UbxParser::new();
        let (other, m) = frame(0x0A, 0x04, 40);
        assert_eq!(count_messages(&mut parser, &other[..m]), 0);
        let (good, n) = frame(UBX_CLASS_NAV, UBX_ID_NAV_PVT, UBX_NAV_PVT_LEN);
        assert_eq!(count_messages(&mut parser, &good[..n]), 1);
    }
}
