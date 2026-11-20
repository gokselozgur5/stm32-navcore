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
