use stm32_navcore::sensors::icm42688::Icm42688;

#[test]
fn fifo_packet4_extension_bytes_follow_timestamp() {
    let mut pkt = [0u8; 20];
    pkt[0] = 0x78;
    pkt[1..3].copy_from_slice(&0x1234i16.to_be_bytes());
    pkt[7..9].copy_from_slice(&0x0ABCi16.to_be_bytes());
    pkt[15..17].copy_from_slice(&0xFFFFu16.to_be_bytes());
    pkt[17] = 0x4E;
    let s = Icm42688::parse_fifo_packet_20bit(&pkt).unwrap();
    assert_eq!(s.accel_20bit, [0x12344, 0, 0]);
    assert_eq!(s.gyro_20bit, [0x0ABCE, 0, 0]);
}
