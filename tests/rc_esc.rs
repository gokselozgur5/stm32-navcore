use stm32_navcore::protocol::rc_esc::{crc8_dvb_s2, DshotFrame};

#[test]
fn test_dshot_known_answer_1046() {
    let dshot = DshotFrame::new(1046, false);
    assert_eq!(dshot.encode(), 0x82C6);
    let decoded = DshotFrame::decode(0x82C6).unwrap();
    assert_eq!(decoded.throttle, 1046);
    assert!(!decoded.telemetry);
}

#[test]
fn test_crc8_dvb_s2_known_answer_vector() {
    assert_eq!(crc8_dvb_s2(b"123456789"), 0xBC);
}

use stm32_navcore::protocol::rc_esc::{CrsfError, CrsfParser};

fn make_crsf_test_frame() -> ([u8; 26], [u16; 16]) {
    let mut f = [0u8; 26];
    f[..3].copy_from_slice(&[0xC8, 24, 0x16]);
    let ch = [
        172u16, 992, 1811, 500, 1500, 300, 1700, 800, 1200, 400, 1600, 600, 1400, 700, 1300, 1000,
    ];
    for (i, &c) in ch.iter().enumerate() {
        for b in 0..11 {
            if (c & (1 << b)) != 0 {
                f[3 + (i * 11 + b) / 8] |= 1 << ((i * 11 + b) % 8);
            }
        }
    }
    f[25] = crc8_dvb_s2(&f[2..25]);
    (f, ch)
}

#[test]
fn test_crsf_parse_frame_distinct_channels() {
    let (frame, ch) = make_crsf_test_frame();
    let parsed = CrsfParser::parse_frame(&frame).unwrap();
    assert_eq!(parsed.raw_channels, ch);
}

#[test]
fn test_crsf_push_byte_garbage_and_split_frame() {
    let (frame, ch) = make_crsf_test_frame();
    let mut parser = CrsfParser::new();
    assert_eq!(parser.push_byte(0x55), Ok(None));
    assert_eq!(parser.push_byte(0xAA), Ok(None));
    let mut bad_frame = [0u8; 6];
    bad_frame[0] = 0xC8;
    bad_frame[1] = 4;
    bad_frame[2] = 0x28;
    for &b in &bad_frame[..5] {
        assert_eq!(parser.push_byte(b), Ok(None));
    }
    assert_eq!(parser.push_byte(0x00), Err(CrsfError::UnsupportedFrameType));
    for &b in &frame[..10] {
        assert_eq!(parser.push_byte(b), Ok(None));
    }
    let mut parsed = None;
    for &b in &frame[10..] {
        if let Ok(Some(f)) = parser.push_byte(b) {
            parsed = Some(f);
        }
    }
    assert_eq!(parsed.unwrap().raw_channels, ch);
}
