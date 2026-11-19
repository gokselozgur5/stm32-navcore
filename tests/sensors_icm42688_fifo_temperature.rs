use stm32_navcore::sensors::icm42688::Icm42688;

// ICM-42688-P datasheet DS-000347, section 6.1, FIFO packet 3 (16 bytes):
// byte 13 is the 8-bit FIFO_TEMP_DATA, bytes 14..15 are the timestamp.
// Section 4.13: T[degC] = FIFO_TEMP_DATA / 2.07 + 25.
#[test]
fn fifo_packet3_temperature_is_one_byte() {
    let mut pkt = [0u8; 16];
    pkt[0] = 0x68;
    pkt[13] = 20;
    pkt[14..16].copy_from_slice(&0x1234u16.to_be_bytes());
    let raw = Icm42688::parse_fifo_packet_16bit(&pkt).unwrap();
    let temp_c = Icm42688::new().scale_sample(&raw).temp_c;
    assert!((temp_c - 34.66).abs() < 0.1, "temp_c = {temp_c}");
}
