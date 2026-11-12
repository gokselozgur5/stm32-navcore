use stm32_navcore::protocol::{CanFrame20B, NavTelemetry, ProtocolError};

#[test]
fn frame_limits() {
    assert!(CanFrame20B::new(CanFrame20B::MAX_EXT_ID, 8, [0; 8]).is_ok());
    assert_eq!(
        CanFrame20B::new(CanFrame20B::MAX_EXT_ID + 1, 0, [0; 8]),
        Err(ProtocolError::InvalidCanId)
    );
    assert!(CanFrame20B::new(0, 0, [0; 8]).is_ok());
    assert_eq!(CanFrame20B::new(0, 9, [0; 8]), Err(ProtocolError::InvalidCanDlc));
}

fn sample() -> NavTelemetry {
    NavTelemetry {
        timestamp_ms: 123_456,
        pos_ned_m: [10.0, -20.0, 5.5],
        vel_ned_mps: [1.0, 2.0, 3.0],
        euler_rad: [0.0, 0.5, -1.0],
        status: 3,
    }
}

#[test]
fn pack_layout_and_roundtrip() {
    let f = sample().pack_can().unwrap();
    for (i, fr) in f.iter().enumerate() {
        assert_eq!(fr.id, NavTelemetry::CAN_BASE_ID + i as u32);
        assert_eq!(fr.dlc, if i == 5 { 3 } else { 8 });
    }
    assert_eq!(NavTelemetry::unpack_can(&f).unwrap(), sample());
}
