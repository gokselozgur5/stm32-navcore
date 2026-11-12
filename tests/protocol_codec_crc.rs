use stm32_navcore::protocol::{crc16_ccitt, NavTelemetry, ProtocolError};

#[test]
fn crc16_known_answers() {
    assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    assert_eq!(crc16_ccitt(&[0x00]), 0xE1F0);
    assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
}

#[test]
fn crc16_appended_big_endian_yields_zero() {
    let mut m = b"123456789".to_vec();
    m.extend_from_slice(&0x29B1u16.to_be_bytes());
    assert_eq!(crc16_ccitt(&m), 0);
}
