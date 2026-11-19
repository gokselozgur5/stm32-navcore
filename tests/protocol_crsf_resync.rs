use stm32_navcore::protocol::rc_esc::{crc8_dvb_s2, CrsfParser};

fn rc_frame(c0: u16, c1: u16) -> [u8; 26] {
    let mut f = [0u8; 26];
    f[..3].copy_from_slice(&[0xC8, 24, 0x16]);
    let ch = [c0, c1];
    for (i, &c) in ch.iter().enumerate() {
        for b in 0..11 {
            if c & (1 << b) != 0 {
                f[3 + (i * 11 + b) / 8] |= 1 << ((i * 11 + b) % 8);
            }
        }
    }
    f[25] = crc8_dvb_s2(&f[2..25]);
    f
}

#[test]
fn crsf_parser_recovers_after_one_lost_byte() {
    let frame = rc_frame(200, 995);
    let mut parser = CrsfParser::new();
    let mut decoded = 0;
    for n in 0..50 {
        for &b in &frame[usize::from(n == 0)..] {
            decoded += usize::from(matches!(parser.push_byte(b), Ok(Some(_))));
        }
    }
    assert!(decoded >= 40, "decoded {decoded} of 49 complete frames");
}
