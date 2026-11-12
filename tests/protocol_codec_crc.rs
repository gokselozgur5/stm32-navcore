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

fn sample() -> NavTelemetry {
    NavTelemetry {
        timestamp_ms: 0x0100_0000,
        pos_ned_m: [1.5, -2.5, 0.0],
        vel_ned_mps: [0.0, 3.25, -0.5],
        euler_rad: [0.1, -0.2, 0.0],
        status: 0,
    }
}

#[test]
fn uart_roundtrip_ends_with_delimiter() {
    let mut buf = [0u8; NavTelemetry::MAX_UART_FRAME_LEN];
    let n = sample().pack_uart(&mut buf).unwrap();
    assert_eq!(buf[n - 1], 0);
    assert!(!buf[..n - 1].contains(&0));
    assert_eq!(NavTelemetry::unpack_uart(&buf[..n]).unwrap(), sample());
}

#[test]
fn uart_rejects_every_single_bit_flip() {
    let mut buf = [0u8; NavTelemetry::MAX_UART_FRAME_LEN];
    let n = sample().pack_uart(&mut buf).unwrap();
    for i in 0..n - 1 {
        for bit in 0..8 {
            let mut f = buf;
            f[i] ^= 1 << bit;
            assert!(
                NavTelemetry::unpack_uart(&f[..n]).is_err(),
                "byte {i} bit {bit}"
            );
        }
    }
}

#[test]
fn uart_rejects_truncated_and_small_buffer() {
    let mut buf = [0u8; NavTelemetry::MAX_UART_FRAME_LEN];
    let n = sample().pack_uart(&mut buf).unwrap();
    assert!(NavTelemetry::unpack_uart(&buf[..n - 3]).is_err());
    let mut small = [0u8; 10];
    assert_eq!(
        sample().pack_uart(&mut small),
        Err(ProtocolError::BufferTooSmall)
    );
}
