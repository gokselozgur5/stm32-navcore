use stm32_navcore::ubx::*;

fn relposned_frame() -> [u8; 72] {
    let mut f = [0u8; 72];
    f[..4].copy_from_slice(&[UBX_SYNC_1, UBX_SYNC_2, UBX_CLASS_NAV, UBX_ID_NAV_RELPOSNED]);
    f[4] = 64;
    let (a, b) = ubx_checksum(&f[2..70]);
    f[70] = a;
    f[71] = b;
    f
}

#[test]
fn frame_inside_rejected_frame_is_available_without_more_input() {
    let inner = relposned_frame();
    let mut outer = [0u8; 80];
    outer[..4].copy_from_slice(&[UBX_SYNC_1, UBX_SYNC_2, 0x05, 0x01]);
    outer[4] = 72;
    outer[6..78].copy_from_slice(&inner);
    let (a, b) = ubx_checksum(&outer[2..78]);
    outer[78..].copy_from_slice(&[a ^ 0xFF, b]);
    let mut p = UbxParser::new();
    for &byte in &outer[..79] {
        assert_eq!(p.process_byte(byte), Ok(None));
    }
    assert!(p.process_byte(outer[79]).is_err());
    let msg = p.drain().unwrap();
    assert!(matches!(msg, Some(UbxMessage::NavRelPosNed(_))));
}
